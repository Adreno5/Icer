package adreno.turneler.navigation

import adreno.turneler.navigation.IcerParameter.*
import kotlin.math.ceil

data class IcerRouteSample(val tick: Long, val route: IcerRoute)

/** Owned by the live tick thread. Snapshots contain raw routes, never previously averaged output. */
class IcerRouteHistory {
    private val samples = ArrayDeque<IcerRouteSample>()

    fun clear() = samples.clear()

    fun record(tick: Long, route: IcerRoute?, windowTicks: Int = 20) {
        if (samples.lastOrNull()?.tick?.let { tick < it } == true) clear()
        while (samples.firstOrNull()?.tick?.let { it < tick - windowTicks + 1 } == true) samples.removeFirst()
        if (samples.lastOrNull()?.tick == tick) samples.removeLast()
        if (route != null) samples.addLast(IcerRouteSample(tick, route))
    }

    /** A fresh route will occupy [tick]; only the preceding window-1 ticks contribute history. */
    fun snapshot(tick: Long, windowTicks: Int = 20): List<IcerRouteSample> =
        samples.filter { it.tick >= tick - windowTicks + 1 && it.tick < tick }
}

/** Average raw tick samples until each point matures; retain stable geometry and replace only its tail. */
object IcerRouteAverager {
    private data class MeanPoint(val distance: Double, val point: Point2, val ticks: Int, val raw: Point2)

    fun average(fresh: IcerRoute, tick: Long, history: List<IcerRouteSample>,
        environment: NavigationEnvironment, settings: IcerSettings = IcerSettings.DEFAULT,
        incumbent: IcerRoute? = null): IcerRoute {
        val window = settings.int(HISTORY_TICKS)
        if (window == 1) return fresh
        val start = fresh.path.start
        val direction = fresh.tangent(0.0)
        val distanceLimit = settings[INCUMBENT_DISTANCE]
        val retained = incumbent?.takeIf { it.frozenStart || it.frozenTailPoints.isNotEmpty() }?.let { route ->
            val projection = route.project(start, (start - route.origin).length())
            if (projection.distance > distanceLimit || projection.tangent.dot(direction) <= 0.0) null else projection
        }
        val stable = retained?.takeIf { it.s < incumbent.frozenLength - 1e-9 ||
            (it.s <= 1e-9 && incumbent.frozenStart) }
        val prefix = if (stable != null) incumbent.frozenSuffix(stable.s) else emptyList()
        val anchor = if (stable != null) prefix.lastOrNull()?.p3 ?: stable.point else null
        // Stable prefix points and their handles never enter a new fit. Only travelled geometry is trimmed.
        if (stable != null && (!environment.clear(fresh.origin, stable.point) ||
                !environment.ice(stable.point) || !prefix.all { safe(it, environment) })) return fresh
        val offset = anchor?.let { fresh.project(it, (it - start).length()).s } ?: 0.0
        val fixed = if (retained == null) emptyList() else incumbent.frozenTailPoints.filter { it.s >= retained.s - 1e-9 }
        if (fixed.any { !environment.ice(it.point) }) return fresh
        // A held raw route counts once for every tick it was held; group only to avoid re-projecting it.
        val aligned = history.filter { it.tick >= tick - window + 1 && it.tick < tick }
            .distinctBy { it.tick }.groupingBy { it.route }.eachCount().mapNotNull { (route, weight) ->
                val projection = route.project(start, (start - route.origin).length())
                if (projection.distance > distanceLimit || projection.tangent.dot(direction) <= 0.0) null
                else Triple(route, projection.s, weight)
            }
        if (aligned.isEmpty() && stable == null && fixed.isEmpty()) return fresh
        val remaining = fresh.length - offset
        if (remaining <= 1e-9 && prefix.isNotEmpty() && fixed.isEmpty()) {
            return IcerRoute.profiled(BezierPath(prefix), fresh.origin, environment,
                settings[WIDTH_LIMIT], fresh.reachesHorizon, fresh.paintedHalfWidth, prefix.size)
        }
        val count = ceil(remaining / settings[HISTORY_SPACING]).toInt().coerceAtLeast(1)
        val means = (0..count).map { index ->
            if (index == 0 && anchor != null) {
                return@map MeanPoint(offset, anchor, window, anchor)
            }
            val distance = offset + remaining * index / count
            val current = fresh.point(distance)
            val tangent = fresh.tangent(distance)
            var sum = current
            var weight = 1
            for ((route, offset, ticks) in aligned) {
                val s = offset + distance
                // Ended horizons contribute nothing beyond their end; clamping would drag the tail back.
                if (s > route.length + 1e-9) continue
                val old = route.point(s)
                if ((old - current).length() > distanceLimit || route.tangent(s).dot(tangent) <= 0.0) continue
                if (!environment.ice(old)) continue
                sum += old * ticks.toDouble()
                weight += ticks
            }
            MeanPoint(distance, sum * (1.0 / weight), weight, current)
        }.toMutableList()
        // Agreement can fail at an earlier point and resume later. Such mature knots also stay fixed.
        var fixedDistance = offset
        val fixedMeans = fixed.map { old ->
            fixedDistance = fresh.project(old.point, old.s - retained!!.s).s.coerceAtLeast(fixedDistance)
            MeanPoint(fixedDistance, old.point, window, old.point)
        }
        means.removeAll { candidate -> candidate.point != anchor &&
            fixedMeans.any { kotlin.math.abs(it.distance - candidate.distance) <= 1e-9 } }
        means.addAll(fixedMeans)
        means.sortBy { it.distance }
        val points = means.map { it.point }
        val weights = means.map { it.ticks }.toIntArray()
        fun fit(tail: List<Point2>, ticks: IntArray): IcerRoute? {
            // The mean is always formed from raw history. Reusing a frozen prefix is not recursive averaging.
            val tangents = tail.indices.map { i ->
                when (i) {
                    0 -> tail[1] - tail[0]
                    tail.lastIndex -> tail[i] - tail[i - 1]
                    else -> (tail[i + 1] - tail[i - 1]) * 0.5
                }
            }
            val joinHandle = prefix.lastOrNull()?.let { it.p3 - it.p2 }
            for (scale in listOf(1.0, 0.75, 0.5, 0.25, 0.0)) {
                val curves = (0 until tail.lastIndex).map { i ->
                    val outgoing = if (i == 0 && joinHandle != null) joinHandle else tangents[i] * (scale / 3.0)
                    CubicBezier(tail[i], tail[i] + outgoing,
                        tail[i + 1] - tangents[i + 1] * (scale / 3.0), tail[i + 1])
                }
                if (!environment.clear(fresh.origin, prefix.firstOrNull()?.p0 ?: tail.first()) ||
                    !curves.all { safe(it, environment) }) continue
                // Endpoints need a full window of contributing ticks before their cubic can freeze.
                val mature = (0 until tail.lastIndex).takeWhile { ticks[it] >= window && ticks[it + 1] >= window }.size
                return IcerRoute.profiled(BezierPath(prefix + curves), fresh.origin, environment,
                    settings[WIDTH_LIMIT], fresh.reachesHorizon, fresh.paintedHalfWidth,
                    prefix.size + mature, ticks[0] >= window,
                    tail.indices.filter { it > mature && ticks[it] >= window }.map { prefix.size + it })
            }
            return null
        }
        fit(points, weights)?.let { return it }
        // Two individually valid routes can average across a dry island or an obstacle.
        // A safe stable prefix remains fixed even if the mean tail cannot be joined safely.
        if (stable != null || fixed.isNotEmpty()) {
            val rawPoints = means.map { it.raw }
            val rawTicks = means.map { if (it.point == it.raw && it.ticks >= window) window else 1 }.toIntArray()
            fit(rawPoints, rawTicks)?.let { return it }
            if (fixed.isNotEmpty()) {
                val (index, suffix) = incumbent!!.suffix(retained!!.s)
                if (environment.clear(fresh.origin, suffix.first().p0) && suffix.all { safe(it, environment) }) {
                    val frozen = (incumbent.frozenSegments - index).coerceAtLeast(0)
                    return IcerRoute.profiled(BezierPath(suffix), fresh.origin, environment,
                        settings[WIDTH_LIMIT], false, fresh.paintedHalfWidth, frozen,
                        frozen > 0 || (retained.s <= 1e-9 && incumbent.frozenStart),
                        fixed.map { it.index - index })
                }
            }
            if (prefix.isNotEmpty()) return IcerRoute.profiled(BezierPath(prefix), fresh.origin, environment,
                settings[WIDTH_LIMIT], false, fresh.paintedHalfWidth, prefix.size)
        }
        return fresh
    }

    private fun safe(curve: CubicBezier, environment: NavigationEnvironment): Boolean {
        val polygon = (curve.p1 - curve.p0).length() + (curve.p2 - curve.p1).length() +
            (curve.p3 - curve.p2).length()
        val samples = ceil(polygon / 0.2).toInt().coerceAtLeast(2)
        var previous = curve.p0
        return environment.ice(previous) && (1..samples).all { i ->
            val point = curve.point(i.toDouble() / samples)
            val safe = environment.ice(point) && environment.clear(previous, point)
            previous = point
            safe
        }
    }
}
