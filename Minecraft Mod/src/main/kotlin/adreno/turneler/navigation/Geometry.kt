package adreno.turneler.navigation

import kotlin.math.*

data class Point2(
    val x: Double,
    val z: Double,
) {
    operator fun plus(other: Point2) = Point2(x + other.x, z + other.z)

    operator fun minus(other: Point2) = Point2(x - other.x, z - other.z)

    operator fun times(scale: Double) = Point2(x * scale, z * scale)

    fun dot(other: Point2) = x * other.x + z * other.z

    fun cross(other: Point2) = x * other.z - z * other.x

    fun lengthSquared() = dot(this)

    fun length() = sqrt(lengthSquared())

    fun unit(fallback: Point2 = Point2(0.0, 1.0)): Point2 = if (lengthSquared() > 1e-12) this * (1.0 / length()) else fallback

    fun normal() = Point2(-z, x)

    fun rotate(angle: Double) = this * cos(angle) + normal() * sin(angle)

    fun angleTo(other: Point2) = atan2(cross(other), dot(other))

    companion object {
        val ZERO = Point2(0.0, 0.0)

        fun facing(yaw: Double) = Point2(-sin(yaw), cos(yaw))
    }
}

data class CubicBezier(
    val p0: Point2,
    val p1: Point2,
    val p2: Point2,
    val p3: Point2,
) {
    fun suffix(t: Double): CubicBezier {
        val a = p0 * (1 - t) + p1 * t
        val b = p1 * (1 - t) + p2 * t
        val c = p2 * (1 - t) + p3 * t
        val d = a * (1 - t) + b * t
        val e = b * (1 - t) + c * t
        return CubicBezier(d * (1 - t) + e * t, e, c, p3)
    }

    fun point(t: Double): Point2 {
        val u = 1.0 - t
        return p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t)
    }

    fun derivative(t: Double): Point2 {
        val u = 1.0 - t
        return (p1 - p0) * (3 * u * u) + (p2 - p1) * (6 * u * t) + (p3 - p2) * (3 * t * t)
    }

    fun secondDerivative(t: Double) = (p2 - p1 * 2.0 + p0) * (6 * (1 - t)) + (p3 - p2 * 2.0 + p1) * (6 * t)

    fun curvature(t: Double): Double {
        val d = derivative(t)
        return d.cross(secondDerivative(t)) / max(1e-9, d.lengthSquared().pow(1.5))
    }
}

/**
 * A route: one or more cubic segments joined end to end. The parameter is global and runs 0..1 across the
 * whole chain, each segment owning the fraction of it its own length earns; a path of one segment behaves
 * exactly as it did when a path *was* one cubic, which is what keeps this change behavior-preserving.
 */
class BezierPath(
    val segments: List<CubicBezier>,
) {
    constructor(curve: CubicBezier) : this(listOf(curve))

    private val divisions = 96

    /** Polyline samples, [divisions] per segment, with the joins shared between neighbours. */
    val points: List<Point2> =
        buildList {
            segments.forEachIndexed { index, segment ->
                for (i in (if (index == 0) 0 else 1)..divisions) add(segment.point(i.toDouble() / divisions))
            }
        }

    private val samples = points.size - 1

    /** Cumulative arc length at each sample of [points]. */
    private val lengths = DoubleArray(points.size)

    /** Global parameter each segment starts at. */
    private val starts = DoubleArray(segments.size)

    init {
        for (i in 1 until points.size) lengths[i] = lengths[i - 1] + (points[i] - points[i - 1]).length()
        val total = lengths.last().coerceAtLeast(1e-9)
        for (i in 1 until segments.size) {
            starts[i] = starts[i - 1] + (lengths[i * divisions] - lengths[(i - 1) * divisions]) / total
        }
    }

    val length: Double get() = lengths.last()

    val start: Point2 get() = segments.first().p0

    val end: Point2 get() = segments.last().p3

    /** Heading the route leaves in, which is what a candidate's end is judged on. */
    val endDirection: Point2 get() = segments.last().derivative(1.0).unit()

    private fun spanEnd(index: Int) = if (index + 1 < segments.size) starts[index + 1] else 1.0

    /** Which segment owns global parameter [t], and the parameter inside it. */
    private fun segmentAt(t: Double): Pair<Int, Double> {
        val clamped = t.coerceIn(0.0, 1.0)
        var index = segments.size - 1
        for (i in segments.indices) {
            if (clamped < spanEnd(i)) {
                index = i
                break
            }
        }
        val span = (spanEnd(index) - starts[index]).coerceAtLeast(1e-9)
        return index to (clamped - starts[index]) / span
    }

    private fun globalOf(
        index: Int,
        local: Double,
    ) = starts[index] + local * (spanEnd(index) - starts[index])

    fun point(t: Double): Point2 {
        val (index, local) = segmentAt(t)
        return segments[index].point(local)
    }

    fun tangent(t: Double): Point2 {
        val (index, local) = segmentAt(t)
        return segments[index].derivative(local)
    }

    fun curvature(t: Double): Double {
        val (index, local) = segmentAt(t)
        return segments[index].curvature(local)
    }

    fun distanceAt(t: Double): Double {
        val (index, local) = segmentAt(t)
        val from = index * divisions
        val scaled = local.coerceIn(0.0, 1.0) * divisions
        val i = from + scaled.toInt().coerceAtMost(divisions - 1)
        // The fraction is measured from the *clamped* sample, not from the unclamped one: at t = 1 the
        // scaled index is exactly divides, the clamped one is divisions - 1 behind it, and the difference
        // between the two is the whole arc length of the last sample. Getting this wrong made distanceAt(1)
        // come back a hair short of the length, which is the value the rollout's own end guard compares
        // against: the guard never fired, every candidate ran its full 240 ticks, and the cost was decided
        // by what each plan did after its route instead of what the route is.
        return lengths[i] + (lengths[i + 1] - lengths[i]) * (scaled - (i - from))
    }

    fun parameterAt(distance: Double): Double {
        val d = distance.coerceIn(0.0, length)
        val found = lengths.binarySearch(d)
        val sample = if (found >= 0) found.coerceAtMost(samples - 1) else (-found - 2).coerceIn(0, samples - 1)
        val index = (sample / divisions).coerceIn(0, segments.size - 1)
        val from = index * divisions
        val i = sample.coerceIn(from, from + divisions - 1)
        val fraction = (d - lengths[i]) / max(1e-9, lengths[i + 1] - lengths[i])
        return globalOf(index, ((i - from) + fraction) / divisions)
    }

    fun closest(
        position: Point2,
        minimum: Double = 0.0,
    ): Double {
        var best = minimum.coerceIn(0.0, 1.0)
        var distance = Double.POSITIVE_INFINITY
        val (startIndex, startLocal) = segmentAt(best)
        val from = startIndex * divisions + (startLocal * divisions).toInt().coerceIn(0, divisions - 1)
        for (i in from until samples) {
            val a = points[i]
            val delta = points[i + 1] - a
            val fraction = ((position - a).dot(delta) / max(1e-12, delta.lengthSquared())).coerceIn(0.0, 1.0)
            val t = globalOf(i / divisions, ((i % divisions) + fraction) / divisions).coerceAtLeast(minimum)
            val d = (point(t) - position).lengthSquared()
            if (d < distance) {
                distance = d
                best = t
            }
        }
        return best
    }

    /** The part of the route past parameter [t], as a route of its own. */
    fun suffix(t: Double): BezierPath {
        val (index, local) = segmentAt(t)
        val tail = ArrayList<CubicBezier>(segments.size - index)
        tail += segments[index].suffix(local)
        for (i in index + 1 until segments.size) tail += segments[i]
        return BezierPath(tail)
    }

    /** First forward circle exit, refined on the curve itself, never a parameter behind progress. */
    fun lookAhead(
        position: Point2,
        radius: Double,
        progress: Double,
    ): Double {
        val radiusSquared = radius * radius
        var previous = progress
        var inside = (point(previous) - position).lengthSquared() <= radiusSquared
        for (i in 1..samples) {
            val t = progress + (1.0 - progress) * i / samples
            val nextInside = (point(t) - position).lengthSquared() <= radiusSquared
            if (inside && !nextInside) {
                var low = previous
                var high = t
                repeat(24) {
                    val mid = (low + high) * 0.5
                    if ((point(mid) - position).lengthSquared() <= radiusSquared) low = mid else high = mid
                }
                return (low + high) * 0.5
            }
            previous = t
            inside = nextInside
        }
        return if (inside) 1.0 else parameterAt(distanceAt(progress) + radius)
    }
}
