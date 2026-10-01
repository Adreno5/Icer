package adreno.turneler.navigation

import kotlin.math.*
import adreno.turneler.navigation.IcerParameter.*

data class IcerProjection(
    val point: Point2,
    val tangent: Point2,
    val curvature: Double,
    val s: Double,
    val distance: Double,
)

/** A mature knot beyond the continuously stable prefix. */
data class IcerFrozenPoint(val index: Int, val s: Double, val point: Point2)

/** A local centre line with mature knots. Width and friction belong to this terrain snapshot. */
class IcerRoute(
    val path: BezierPath,
    val origin: Point2,
    private val halfWidths: DoubleArray,
    private val frictions: DoubleArray,
    val reachesHorizon: Boolean = false,
    val paintedHalfWidth: Double? = null,
    val frozenSegments: Int = 0,
    val frozenStart: Boolean = frozenSegments > 0,
    val frozenTailPoints: List<IcerFrozenPoint> = emptyList(),
) {
    private val arcs = path.segments.map { curve ->
        val polygon = (curve.p1 - curve.p0).length() + (curve.p2 - curve.p1).length() +
            (curve.p3 - curve.p2).length()
        val divisions = ceil(polygon * 8.0).toInt().coerceIn(16, 4000)
        DoubleArray(divisions + 1).also { arc ->
            var previous = curve.p0
            for (i in 1..divisions) {
                val point = curve.point(i.toDouble() / divisions)
                arc[i] = arc[i - 1] + (point - previous).length()
                previous = point
            }
        }
    }
    private val offsets = DoubleArray(arcs.size + 1).also { values ->
        for (i in arcs.indices) values[i + 1] = values[i] + arcs[i].last()
    }
    val length: Double get() = offsets.last()
    val frozenLength: Double get() = offsets[frozenSegments.coerceIn(0, arcs.size)]
    private val joints: List<Double> = offsets.drop(1).dropLast(1)

    private fun segmentParameter(s: Double): Pair<Int, Double> {
        val distance = s.coerceIn(0.0, length)
        val found = offsets.binarySearch(distance)
        val index = if (found >= 0) found.coerceAtMost(arcs.lastIndex)
            else (-found - 2).coerceIn(0, arcs.lastIndex)
        val arc = arcs[index]
        val localDistance = distance - offsets[index]
        val sampleFound = arc.binarySearch(localDistance)
        val sample = if (sampleFound >= 0) sampleFound.coerceAtMost(arc.lastIndex - 1)
            else (-sampleFound - 2).coerceIn(0, arc.lastIndex - 1)
        val fraction = ((localDistance - arc[sample]) / (arc[sample + 1] - arc[sample]).coerceAtLeast(1e-9))
            .coerceIn(0.0, 1.0)
        return index to ((sample + fraction) / arc.lastIndex)
    }

    /** Polyline seed, then a bounded Newton solve on each eligible cubic. */
    fun project(position: Point2, previousS: Double = 0.0): IcerProjection {
        var bestPoint = path.start
        var bestTangent = path.segments.first().derivative(0.0).unit()
        var bestS = 0.0
        var bestDistanceSquared = Double.POSITIVE_INFINITY
        val ordered = path.segments.indices.map { segmentIndex ->
            val curve = path.segments[segmentIndex]
            val minX = minOf(curve.p0.x, curve.p1.x, curve.p2.x, curve.p3.x)
            val maxX = maxOf(curve.p0.x, curve.p1.x, curve.p2.x, curve.p3.x)
            val minZ = minOf(curve.p0.z, curve.p1.z, curve.p2.z, curve.p3.z)
            val maxZ = maxOf(curve.p0.z, curve.p1.z, curve.p2.z, curve.p3.z)
            val lowerBound = (position.x - position.x.coerceIn(minX, maxX)).pow(2) +
                (position.z - position.z.coerceIn(minZ, maxZ)).pow(2)
            segmentIndex to lowerBound
        }.sortedBy { it.second }
        for ((segmentIndex, lowerBound) in ordered) {
            if (lowerBound > bestDistanceSquared + 1e-9) continue
            val curve = path.segments[segmentIndex]
            val arc = arcs[segmentIndex]
            val divisions = arc.lastIndex
            var local = 0.0
            var seedDistance = Double.POSITIVE_INFINITY
            for (sample in 0 until divisions) {
                val a = curve.point(sample.toDouble() / divisions)
                val b = curve.point((sample + 1.0) / divisions)
                val edge = b - a
                val fraction = ((position - a).dot(edge) / edge.lengthSquared().coerceAtLeast(1e-12))
                    .coerceIn(0.0, 1.0)
                val d = (a + edge * fraction - position).lengthSquared()
                if (d < seedDistance) {
                    seedDistance = d
                    local = (sample + fraction) / divisions
                }
            }
            repeat(12) {
                val delta = curve.point(local) - position
                val derivative = curve.derivative(local)
                val gradient = delta.dot(derivative)
                val slope = derivative.lengthSquared() + delta.dot(curve.secondDerivative(local))
                if (abs(slope) > 1e-9) local = (local - (gradient / slope).coerceIn(-0.2, 0.2)).coerceIn(0.0, 1.0)
            }
            for (candidate in listOf(0.0, local, 1.0)) {
                val point = curve.point(candidate)
                val d = (point - position).lengthSquared()
                val scaled = candidate * divisions
                val sample = scaled.toInt().coerceAtMost(divisions - 1)
                val s = offsets[segmentIndex] + arc[sample] +
                    (arc[sample + 1] - arc[sample]) * (scaled - sample)
                if (d < bestDistanceSquared - 1e-9 ||
                    (abs(d - bestDistanceSquared) <= 1e-9 && abs(s - previousS) < abs(bestS - previousS))
                ) {
                    bestPoint = point
                    bestTangent = curve.derivative(candidate).unit(bestTangent)
                    bestS = s
                    bestDistanceSquared = d
                }
            }
        }
        return IcerProjection(bestPoint, bestTangent, curvature(bestS), bestS, sqrt(bestDistanceSquared))
    }

    private fun sample(values: DoubleArray, s: Double): Double {
        val index = (s.coerceIn(0.0, length) / PROFILE_STEP).coerceIn(0.0, (values.size - 1).toDouble())
        val lower = index.toInt()
        val upper = (lower + 1).coerceAtMost(values.lastIndex)
        return values[lower] + (values[upper] - values[lower]) * (index - lower)
    }

    fun halfWidth(s: Double) = maxOf(sample(halfWidths, s), paintedHalfWidth ?: 0.0, HULL_HALF_WIDTH)

    fun friction(s: Double) = sample(frictions, s)

    fun point(s: Double): Point2 {
        val (index, local) = segmentParameter(s)
        return path.segments[index].point(local)
    }

    /** Trim travelled geometry without refitting the remaining stable cubics. */
    internal fun frozenSuffix(s: Double): List<CubicBezier> {
        if (s >= frozenLength - 1e-9) return emptyList()
        val (index, local) = segmentParameter(s)
        return listOf(path.segments[index].suffix(local)) + path.segments.subList(index + 1, frozenSegments)
    }

    internal fun suffix(s: Double): Pair<Int, List<CubicBezier>> {
        val (index, local) = segmentParameter(s)
        return index to (listOf(path.segments[index].suffix(local)) + path.segments.drop(index + 1))
    }

    fun tangent(s: Double): Point2 {
        val (index, local) = segmentParameter(s)
        return path.segments[index].derivative(local).unit()
    }

    fun curvature(s: Double): Double {
        val (index, local) = segmentParameter(s)
        var curvature = path.segments[index].curvature(local)
        // Short handles at a cubic join can create a spurious curvature spike. Measure the
        // change of direction across a boat width there, as the Icer route model requires.
        for (i in joints.indices) {
            val joint = joints[i]
            if (min((path.segments[i].p3 - path.segments[i].p2).length(),
                    (path.segments[i + 1].p1 - path.segments[i + 1].p0).length()) >= HULL_WIDTH
            ) continue
            val gap = abs(s - joint)
            if (gap >= HULL_WIDTH) continue
            val before = tangent((joint - HULL_HALF_WIDTH).coerceAtLeast(0.0))
            val after = tangent((joint + HULL_HALF_WIDTH).coerceAtMost(length))
            val smooth = before.angleTo(after) / HULL_WIDTH
            val weight = 1.0 - (gap / HULL_WIDTH).pow(4)
            curvature += (smooth - curvature) * weight
        }
        return curvature
    }

    companion object {
        const val HULL_WIDTH = 1.375
        const val HULL_HALF_WIDTH = HULL_WIDTH / 2.0
        const val PROFILE_STEP = 1.0

        /** Width and friction must be measured on the actual output curve, including an averaged one. */
        internal fun profiled(path: BezierPath, origin: Point2, environment: NavigationEnvironment,
            widthLimit: Double, reachesHorizon: Boolean, paintedHalfWidth: Double? = null,
            frozenSegments: Int = 0, frozenStart: Boolean = frozenSegments > 0,
            frozenTailIndices: List<Int> = emptyList()): IcerRoute {
            val geometry = IcerRoute(path, origin, doubleArrayOf(HULL_HALF_WIDTH), doubleArrayOf(0.98))
            val count = ceil(geometry.length / PROFILE_STEP).toInt() + 1
            val halfWidths = DoubleArray(count)
            val frictions = DoubleArray(count)
            fun margin(point: Point2, direction: Point2): Double {
                var reached = 0.0
                while (reached + 0.25 <= widthLimit && environment.ice(point + direction * (reached + 0.25))) {
                    reached += 0.25
                }
                return reached
            }
            for (i in 0 until count) {
                val s = (i * PROFILE_STEP).coerceAtMost(geometry.length)
                val point = geometry.point(s)
                val normal = geometry.tangent(s).normal()
                halfWidths[i] = HULL_HALF_WIDTH + min(margin(point, normal), margin(point, normal * -1.0))
                frictions[i] = environment.friction(point).coerceIn(0.05, 0.999)
            }
            return IcerRoute(path, origin, halfWidths, frictions, reachesHorizon, paintedHalfWidth,
                frozenSegments, frozenStart, frozenTailIndices.map { index ->
                    IcerFrozenPoint(index, geometry.offsets[index],
                        if (index == path.segments.size) path.end else path.segments[index].p0)
                })
        }
    }
}

/**
 * Beam trace through supported, collision-clear ice terrain. At each step, lateral probes
 * pull candidates onto the middle of their local cross-section. Cubics then smooth the trace,
 * and every resulting piece is
 * sampled against the hull footprint and collision adapter before it can be published.
 */
class IcerPathGenerator {
    private var settings = IcerSettings.DEFAULT
    @Volatile var status = "waiting for ice scan"
        private set

    private fun unavailable(reason: String): IcerRoute? {
        status = reason
        return null
    }

    private data class Trace(
        val point: Point2,
        val direction: Point2,
        val centres: List<Point2>,
        val score: Double,
    )

    private data class CrossSection(val centre: Point2, val margin: Double, val shift: Double)

    fun generate(
        state: BoatState,
        environment: NavigationEnvironment,
        incumbent: IcerRoute? = null,
        horizon: Double = 48.0,
        settings: IcerSettings = IcerSettings.DEFAULT,
    ): IcerRoute? {
        this.settings = settings
        val coarse = generateAtStep(state, environment, incumbent, horizon, settings[COARSE_STEP])
        if (coarse?.reachesHorizon == true) return coarse
        val coarseStatus = status
        // Three-block chords can miss the only safe line through a tight, narrow turn.
        // Retry with shorter chords before accepting a truncated trace.
        val fine = generateAtStep(state, environment, incumbent, horizon, settings[FINE_STEP])
        if (fine != null && (coarse == null || fine.length > coarse.length + 0.5)) return fine
        if (coarse != null) status = coarseStatus
        return coarse ?: fine
    }

    private fun generateAtStep(
        state: BoatState,
        environment: NavigationEnvironment,
        incumbent: IcerRoute?,
        horizon: Double,
        step: Double,
    ): IcerRoute? {
        status = "scanning ice"
        val incumbentProjection = incumbent?.project(state.position)
        val guide = if (incumbent != null && incumbentProjection != null && incumbentProjection.distance <= settings[INCUMBENT_DISTANCE]) {
            incumbent.tangent((incumbentProjection.s + settings[GUIDE_DISTANCE]).coerceAtMost(incumbent.length))
        } else null
        val heading = guide ?: if (state.speed > settings[TRAVEL_THRESHOLD]) state.travel else state.forward
        val start = section(state.position, heading, environment, settings[START_SEARCH], step <= settings[FINE_STEP])
            ?: return unavailable("no hull-safe ice at start")
        if (!environment.ice(start.centre) || !environment.clear(state.position, start.centre)) {
            return unavailable("start to centre is blocked")
        }
        var beam = listOf(Trace(start.centre, heading, listOf(start.centre), 0.0))
        var best = beam.first()
        val steps = ceil(horizon.coerceIn(24.0, 72.0) / step).toInt()
        for (depth in 1..steps) {
            val next = ArrayList<Trace>(beam.size * turns.size)
            for (trace in beam) {
                for (angle in turns) {
                    val direction = trace.direction.rotate(Math.toRadians(angle))
                    val guess = trace.point + direction * step
                    val cross = section(guess, direction, environment, settings[SECTION_SEARCH], step <= settings[FINE_STEP]) ?: continue
                    val centre = cross.centre
                    if ((centre - trace.point).length() > step * 1.75 ||
                        !iceLine(trace.point, centre, environment)
                    ) continue
                    if (trace.centres.dropLast(2).any { (it - centre).length() < step * 0.8 }) continue
                    val previousDistance = if (incumbent != null && incumbentProjection != null && incumbentProjection.distance <= settings[INCUMBENT_DISTANCE]) {
                        (incumbent.point((incumbentProjection.s + depth * step).coerceAtMost(incumbent.length)) - centre)
                            .length().coerceAtMost(4.0)
                    } else 0.0
                    val score = trace.score + step +
                        settings[MARGIN_WEIGHT] * cross.margin.coerceAtMost(3.0) -
                        settings[TURN_WEIGHT] * abs(angle) / 35.0 -
                        settings[SHIFT_WEIGHT] * abs(cross.shift) -
                        settings[INCUMBENT_WEIGHT] * previousDistance
                    next += Trace(centre, direction, trace.centres + centre, score)
                }
            }
            if (next.isEmpty()) break
            // Keep spatially distinct headings so an early straight candidate does not crowd out
            // the candidate that sees the next bend.
            val occupied = HashSet<Triple<Int, Int, Int>>()
            beam = next.sortedByDescending { it.score }.filter { trace ->
                occupied.add(Triple(
                    floor(trace.point.x).toInt(), floor(trace.point.z).toInt(),
                    floor(atan2(trace.direction.x, trace.direction.z) / (PI / 8)).toInt(),
                ))
            }.take(settings.int(BEAM))
            best = beam.maxBy { it.score }
        }
        if (best.centres.size < 2) return unavailable("no connected ice ahead")

        val finalists = (listOf(best) + beam).distinctBy { it.centres }
        var path: BezierPath? = null
        var reachesHorizon = false
        fit@ for (trim in 0 until best.centres.size - 1) {
            for (trace in finalists) {
                val kept = trace.centres.size - trim
                if (kept < 2) continue
                val candidate = smooth(trace.centres.take(kept), environment) ?: continue
                path = candidate
                reachesHorizon = kept == steps + 1
                break@fit
            }
        }
        val safePath = path ?: return unavailable("smoothed path hits terrain")
        val route = IcerRoute.profiled(safePath, state.position, environment, settings[WIDTH_LIMIT], reachesHorizon)
        status = "centre line ${"%.0f".format(route.length)} blocks"
        return route
    }

    private fun section(
        guess: Point2,
        heading: Point2,
        environment: NavigationEnvironment,
        search: Double,
        refineMargin: Boolean,
    ): CrossSection? {
        val normal = heading.normal()
        fun findSeed(spacing: Double): Point2? {
            for (i in 0..ceil(search / spacing).toInt()) {
                for (sign in if (i == 0) listOf(0) else listOf(-1, 1)) {
                    val candidate = guess + normal * (sign * i * spacing)
                    if (environment.ice(candidate)) return candidate
                }
            }
            return null
        }
        val found = findSeed(0.5) ?: findSeed(0.125) ?: return null
        val left = iceMargin(found, normal, environment, settings[WIDTH_LIMIT], refineMargin)
        val right = iceMargin(found, normal * -1.0, environment, settings[WIDTH_LIMIT], refineMargin)
        val shift = ((left - right) * 0.5).coerceIn(-settings[CENTRE_SHIFT], settings[CENTRE_SHIFT])
        val centre = found + normal * shift
        if (!environment.ice(centre)) return null
        return CrossSection(centre, min(left, right) + abs(shift), (centre - guess).dot(normal))
    }

    private fun iceMargin(
        point: Point2,
        direction: Point2,
        environment: NavigationEnvironment,
        limit: Double,
        refine: Boolean = false,
    ): Double {
        var reached = 0.0
        while (reached + 0.25 <= limit && environment.ice(point + direction * (reached + 0.25))) {
            reached += 0.25
        }
        if (!refine || reached >= limit) return reached
        var edge = min(limit, reached + 0.25)
        repeat(4) {
            val middle = (reached + edge) * 0.5
            if (environment.ice(point + direction * middle)) reached = middle else edge = middle
        }
        return reached
    }

    private fun iceLine(from: Point2, to: Point2, environment: NavigationEnvironment): Boolean {
        val delta = to - from
        val samples = ceil(delta.length() / 0.25).toInt().coerceAtLeast(1)
        return (0..samples).all { environment.ice(from + delta * (it.toDouble() / samples)) } &&
            environment.clear(from, to)
    }

    private fun smooth(centres: List<Point2>, environment: NavigationEnvironment): BezierPath? {
        val maximumOffsets = centres.indices.map { i ->
            val direction = when (i) {
                0 -> centres[1] - centres[0]
                centres.lastIndex -> centres[i] - centres[i - 1]
                else -> centres[i + 1] - centres[i - 1]
            }.unit()
            val normal = direction.normal()
            val room = min(iceMargin(centres[i], normal, environment, settings[WIDTH_LIMIT]),
                iceMargin(centres[i], normal * -1.0, environment, settings[WIDTH_LIMIT]))
            (room * settings[SMOOTH_ROOM]).coerceIn(settings[SMOOTH_MIN], settings[SMOOTH_MAX])
        }
        // The cross-sections are sampled only three blocks apart. Block edges can move their
        // measured middle left and right without the ice corridor actually bending. Fit a
        // longer curve through filtered anchors, then fall back to shorter spans in tight ice.
        for ((passes, stride) in listOf(settings.int(SMOOTH_PASSES) to settings.int(ANCHOR_STRIDE),
            settings.int(SMOOTH_PASSES) / 2 to max(1, settings.int(ANCHOR_STRIDE) - 1),
            settings.int(SMOOTH_PASSES) / 4 to min(2, settings.int(ANCHOR_STRIDE)),
            min(1, settings.int(SMOOTH_PASSES)) to 1, 0 to 1)) {
            var filtered = centres
            repeat(passes) {
                filtered = filtered.indices.map { i ->
                    when (i) {
                        0, filtered.lastIndex -> filtered[i]
                        else -> (filtered[i - 1] + filtered[i] * 2.0 + filtered[i + 1]) * 0.25
                    }
                }
            }
            val indices = mutableListOf(0)
            var index = stride
            while (index < filtered.lastIndex && filtered.lastIndex - index > stride / 2) {
                indices += index
                index += stride
            }
            indices += filtered.lastIndex
            val anchors = indices.map(filtered::get)
            val tangents = anchors.indices.map { i ->
                when (i) {
                    0 -> anchors[1] - anchors[0]
                    anchors.lastIndex -> anchors[i] - anchors[i - 1]
                    else -> (anchors[i + 1] - anchors[i - 1]) * 0.5
                }
            }
            for (fraction in listOf(1.0, 0.75, 0.5, 0.25)) {
                val segments = (0 until anchors.lastIndex).map { i ->
                    val scale = fraction * settings[HANDLE_SCALE]
                    CubicBezier(anchors[i], anchors[i] + tangents[i] * (scale / 3.0),
                        anchors[i + 1] - tangents[i + 1] * (scale / 3.0), anchors[i + 1])
                }
                val path = BezierPath(segments)
                // A safe curve can still shave the inside of a narrow bend and leave no room
                // for the hull. Keep it close to the measured middle when the ice is narrow.
                if (centres.indices.any { i ->
                        (path.point(path.closest(centres[i])) - centres[i]).length() > maximumOffsets[i]
                    }
                ) continue
                if (segments.all { curve ->
                        val samples = ceil(((curve.p1 - curve.p0).length() +
                            (curve.p2 - curve.p1).length() + (curve.p3 - curve.p2).length()) / 0.2)
                            .toInt().coerceAtLeast(2)
                        var previous = curve.p0
                        (1..samples).all { sample ->
                            val point = curve.point(sample.toDouble() / samples)
                            val safe = environment.ice(point) && environment.clear(previous, point)
                            previous = point
                            safe
                        }
                    }
                ) return path
            }
        }
        return null
    }

    private val turns get() = listOf(-35, -23, -12, 0, 12, 23, 35).map {
        it * settings[TURN_RANGE] / 35.0
    }
}
