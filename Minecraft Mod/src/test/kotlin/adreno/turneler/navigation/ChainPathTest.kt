package adreno.turneler.navigation

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The chain is a refactor of a path that used to be one cubic, so its single-segment behaviour has to be
 * the old behaviour exactly: every ride, every cost and every chosen plan depends on those numbers not
 * moving. These pin the primitives against the closed form and against the sample table.
 */
class ChainPathTest {
    private val curve = CubicBezier(Point2(0.0, 0.0), Point2(0.0, 20.0), Point2(12.0, 48.0), Point2(30.0, 60.0))

    @Test fun singleSegmentChainMatchesTheCubicItWraps() {
        val path = BezierPath(curve)
        for (i in 0..96) {
            val t = i / 96.0
            assertEquals(curve.point(t).x, path.point(t).x, 1e-12, "point x at " + t)
            assertEquals(curve.point(t).z, path.point(t).z, 1e-12, "point z at " + t)
            assertEquals(curve.derivative(t).x, path.tangent(t).x, 1e-12, "tangent x at " + t)
            assertEquals(curve.derivative(t).z, path.tangent(t).z, 1e-12, "tangent z at " + t)
            assertEquals(curve.curvature(t), path.curvature(t), 1e-12, "curvature at " + t)
        }
        assertEquals(curve.p0.x, path.start.x, 1e-12)
        assertEquals(curve.p3.z, path.end.z, 1e-12)
    }

    @Test fun sampleTableAndInversionStillRoundTrip() {
        val path = BezierPath(curve)
        var summed = 0.0
        for (i in 1..96) summed += (curve.point(i / 96.0) - curve.point((i - 1) / 96.0)).length()
        assertEquals(summed, path.length, 1e-9, "length")
        // The rollout's own end guard compares its travelled distance against the length, so distanceAt(1)
        // has to *be* the length: an off-by-one here left it a hair short, the guard never fired, and every
        // candidate's rollout ran its full 240 ticks.
        assertEquals(path.length, path.distanceAt(1.0), 1e-9, "distanceAt(1) is the whole length")
        assertEquals(0.0, path.distanceAt(0.0), 1e-12, "distanceAt(0) is the start")
        for (i in 1 until 40) {
            val distance = path.length * i / 40.0
            assertEquals(distance, path.distanceAt(path.parameterAt(distance)), 1e-6, "round trip at " + distance)
        }
    }

    @Test fun suffixIsTheSameCurveFromThere() {
        val path = BezierPath(curve)
        val cut = path.suffix(0.3)
        assertEquals(1, cut.segments.size)
        assertEquals(curve.suffix(0.3).p0.x, cut.start.x, 1e-12)
        assertEquals(curve.p3.z, cut.end.z, 1e-12)
    }

    @Test fun chainSpanIsLengthProportionalAndContinuous() {
        val first = CubicBezier(Point2(0.0, 0.0), Point2(0.0, 6.0), Point2(0.0, 12.0), Point2(0.0, 20.0))
        val second = CubicBezier(Point2(0.0, 20.0), Point2(0.0, 28.0), Point2(10.0, 34.0), Point2(20.0, 40.0))
        val chain = BezierPath(listOf(first, second))
        val firstLength = BezierPath(first).length
        val secondLength = BezierPath(second).length
        assertEquals(firstLength + secondLength, chain.length, 1e-9, "chain length is the sum")
        val fraction = firstLength / chain.length
        val join = chain.point(fraction)
        assertEquals(first.p3.x, join.x, 1e-6, "join x")
        assertEquals(first.p3.z, join.z, 1e-6, "join z")
        assertTrue((chain.point(fraction + 1e-6) - join).length() < 1e-3, "continuous at the join")
        assertEquals(second.p3.z, chain.point(1.0).z, 1e-9, "chain end")
    }
}
