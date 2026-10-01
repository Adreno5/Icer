package adreno.turneler.navigation

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.*

class IcerControllerTest {
    private fun open(friction: Double = 0.98) = object : NavigationEnvironment {
        override fun iceBlock(x: Int, z: Int) = true
        override fun ice(position: Point2) = true
        override fun clear(from: Point2, to: Point2) = true
        override fun friction(position: Point2) = friction
    }

    private fun line(length: Double = 1000.0, width: Double = 3.0) = IcerRoute(
        BezierPath(CubicBezier(Point2.ZERO, Point2(0.0, length / 3.0),
            Point2(0.0, length * 2.0 / 3.0), Point2(0.0, length))),
        Point2.ZERO, doubleArrayOf(width), doubleArrayOf(0.98), true)

    private fun bend(radius: Double = 18.0, width: Double = 3.0, approach: Double = 0.0,
        paintedWidth: Double? = null): IcerRoute {
        val start = Point2(0.0, approach)
        val handle = radius * 4.0 / 3.0 * tan(PI / 8.0)
        val curves = mutableListOf<CubicBezier>()
        if (approach > 0.0) curves += CubicBezier(Point2.ZERO, Point2(0.0, approach / 3.0),
            Point2(0.0, approach * 2.0 / 3.0), start)
        curves += CubicBezier(start, start + Point2(0.0, handle),
            start + Point2(radius - handle, radius), start + Point2(radius, radius))
        return IcerRoute(BezierPath(curves), Point2.ZERO, doubleArrayOf(width), doubleArrayOf(0.98),
            paintedHalfWidth = paintedWidth)
    }

    @Test fun narrowStraightKeepsFullForwardThrustAtHighSpeedOnBothIceFrictions() {
        val route = line(width = 1.5)
        for (friction in listOf(0.98, 0.989)) {
            val ice = open(friction)
            val controller = IcerController()
            var state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
            repeat(180) { tick ->
                val measured = IcerPhysics.damp(state, friction)
                val input = controller.next(measured, ice, route, tick = tick.toLong())
                assertEquals(BoatInput(false, false, true, false), input, "F=$friction tick=$tick")
                assertEquals(0.0, controller.brakingDemand)
                state = IcerPhysics.moveAfterDamping(measured, input)
            }
            assertTrue(state.speed > if (friction == 0.98) 1.9 else 3.0, "F=$friction speed=${state.speed}")
        }
    }

    @Test fun bendsOutsideTheCoastingWindowDoNotSuppressStraightThrust() {
        val route = bend(approach = 80.0)
        val state = BoatState(Point2.ZERO, Point2(0.0, 1.5), 0.0, 0.0)
        val controller = IcerController()
        val input = controller.next(state, open(), route)
        assertEquals(0.0, controller.brakingDemand)
        assertTrue(input.forward)
        // Blue ice's longer drag tail reaches the same future corner sooner.
        controller.next(state, open(0.989), route)
        assertTrue(controller.brakingDemand!! > 0.0)
    }

    @Test fun brakingSubtractsTheNaturalDragInsteadOfApplyingItTwice() {
        val route = bend()
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.8), 0.0, 0.0)
        val iceController = IcerController()
        val blueController = IcerController()
        iceController.next(state, open(), route)
        blueController.next(state, open(0.989), route)
        val iceBrake = iceController.brakingDemand!!
        val blueBrake = blueController.brakingDemand!!
        assertTrue(iceBrake in 0.0..0.9 && blueBrake in 0.0..0.9)
        assertEquals(0.8 * (0.989 - 0.98) / 0.08, blueBrake - iceBrake, 1e-9,
            "the same near corner needs the same net deceleration; blue ice contributes less drag")
    }

    @Test fun averagingAPathologicalCurvatureSpikeCannotReverseTheRequestedSide() {
        val path = BezierPath(CubicBezier(Point2.ZERO, Point2(0.0, 0.01),
            Point2(5.0, 8.0), Point2(12.0, 8.0)))
        val route = IcerRoute(path, Point2.ZERO, doubleArrayOf(3.0), doubleArrayOf(0.98))
        val controller = IcerController()
        controller.next(BoatState(Point2.ZERO, Point2(0.0, 0.6), 0.0, 0.0), open(), route)
        val decision = controller.decision!!
        assertTrue(route.curvature(0.0) < -1.0)
        assertTrue(decision.steeringCurvature < 0.0)
        assertTrue(decision.requestedSide < 0.0, "$decision")
        assertTrue(decision.steeringSide < 0.0, "$decision")
    }

    @Test fun wideMeasuredIceStrengthensFeedbackInsteadOfAllowingLongSidewaysDrift() {
        val state = BoatState(Point2(2.0, 5.0), Point2(0.2, 0.5), 0.0, 0.0)
        val controller = IcerController()
        controller.next(state, open(), line(width = 3.0))
        val narrowSide = controller.decision!!.requestedSide
        controller.next(state, open(), line(width = 9.0))
        val wideSide = controller.decision!!.requestedSide
        assertTrue(narrowSide > 0.0)
        assertTrue(wideSide > narrowSide, "wide=$wideSide narrow=$narrowSide")
    }

    @Test fun yawTracksABoundedRateWithNeutralTicksAndPredictiveCounterSteering() {
        val controller = IcerController()
        assertEquals(0, controller.turnCommand(0.0, 0.7, 2.0, 0.0, 0.0))
        assertEquals(-1, controller.turnCommand(0.0, 7.0, 2.0, 0.0, 0.0))
        assertEquals(1, controller.turnCommand(0.0, -7.0, -2.0, 0.0, 0.0))
        assertEquals(1, controller.turnCommand(179.0, 0.0, -171.0, 0.0, 0.0))
        val route = bend(radius = 3.0)
        val state = BoatState(Point2.ZERO, Point2(0.0, 2.0), 0.0, 0.0)
        controller.next(state, open(), route)
        val forwardRate = controller.decision!!.referenceRate
        controller.next(state.copy(velocity = state.velocity * -1.0), open(), route)
        val reverseRate = controller.decision!!.referenceRate
        assertEquals(-5.0, Math.toDegrees(forwardRate), 1e-9)
        assertTrue(reverseRate > 0.0, "backwards motion must reverse the route reference rate")
        assertTrue(Math.toDegrees(reverseRate) <= 5.0)
    }

    @Test fun turnForecastUsesTheFirstAlreadyDampedYawRateWithoutRoundingYawTooEarly() {
        val settings = IcerSettings(mapOf("forecast_min" to 1.0, "forecast_angle" to 5.0))
        val route = line()
        val controller = IcerController()
        val state = BoatState(Point2.ZERO, Point2.ZERO, Math.toRadians(20.0), Math.toRadians(0.75))
        controller.next(state, open(), route, settings)
        // A 2 degree error requests only .4 deg/tick: .75 is inside the neutral rate zone.
        val yaw = Math.toRadians((20.0f + 0.75f).toDouble())
        val firstSide = 0.04 * IcerPhysics.facing(yaw).dot(Point2(-1.0, 0.0))
        val duty = controller.cumulativeTurnDuty(state, 0.98, Math.toRadians(22.0), 0.0,
            0.0, Point2(-1.0, 0.0), firstSide * 0.5)
        assertEquals(0.5, duty, 1e-12)
    }

    @Test fun previewProgressDoesNotRetreatWhenTheBoatMovesBackOnTheSameRoute() {
        val route = bend()
        val controller = IcerController()
        val ice = open()
        fun state(s: Double): BoatState {
            val tangent = route.tangent(s)
            return BoatState(route.point(s), tangent * 0.4, atan2(-tangent.x, tangent.z), 0.0)
        }
        controller.next(state(10.0), ice, route)
        val ahead = controller.decision!!.targetProgress
        controller.next(state(3.0), ice, route)
        assertTrue(controller.decision!!.targetProgress >= ahead)
        assertEquals(3.0, controller.decision!!.progress, 1e-5)
    }

    @Test fun coastingStillSteersTowardTheBestForwardThrustHeading() {
        val controller = IcerController()
        val input = controller.next(BoatState(Point2.ZERO, Point2(0.0, 0.3), 0.0, Math.toRadians(1.5)),
            open(), bend(radius = 3.0, width = 1.2, approach = 5.0))
        val decision = controller.decision!!
        assertFalse(decision.useW)
        assertFalse(input.forward)
        val forwardYaw = asin((decision.steeringSide / 0.04).coerceIn(-1.0, 1.0))
        assertTrue(abs(decision.targetYaw - forwardYaw) > 0.1, "$decision")
        assertTrue(input.left, "coasting must retain heading feedback")
    }

    @Test fun reversePulsesReleaseOncePerRealTickPeriodWithoutCancellingSteering() {
        for (slow in listOf(true, false)) {
            val controller = IcerController()
            val route = bend(radius = 3.0, width = 1.2, approach = if (slow) 0.0 else 20.0)
            val state = BoatState(Point2.ZERO, Point2(0.0, if (slow) 0.3 else 1.0),
                Math.toRadians(if (slow) 5.0 else 0.0), Math.toRadians(1.5))
            val period = if (slow) 10 else 5
            val inputs = (0 until 20).map { tick -> controller.next(state, open(), route, tick = tick.toLong()) }
            for ((tick, input) in inputs.withIndex()) {
                assertFalse(input.forward, "tick=$tick $input")
                assertEquals(tick % period != period - 1, input.backward, "slow=$slow tick=$tick $input")
                assertTrue(input.left || input.right, "S release must retain A/D at tick=$tick")
            }
            // A worker replacing the route must not restart the reverse pulse clock.
            val refreshed = bend(radius = 3.0, width = 1.2, approach = if (slow) 0.0 else 20.0)
            assertFalse(controller.next(state, open(), refreshed, tick = 29L).backward)
        }
    }

    @Test fun paintedCoastBrakingSuppressesReverseButKeepsMeasuredYawFeedback() {
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.5), 0.0, Math.toRadians(1.5))
        val controller = IcerController()
        val painted = controller.next(state, open(), bend(radius = 3.0, width = 1.2,
            approach = 10.0, paintedWidth = 1.0))
        assertTrue(controller.decision!!.coastBrake)
        assertEquals(BoatInput(true, false, false, false), painted)
        val unpainted = controller.next(state, open(), bend(radius = 3.0, width = 1.2, approach = 10.0))
        assertFalse(controller.decision!!.coastBrake)
        assertTrue(unpainted.backward)
        assertTrue(unpainted.left)
        val moved = IcerPhysics.moveAfterDamping(state, painted)
        assertEquals(0.005f.toDouble(), moved.acceleration.length(), 5e-7,
            "releasing S while steering still produces vanilla bare-turn thrust")
    }

    @Test fun newFeedbackAndRateParametersReachTheController() {
        val controller = IcerController()
        val ice = open()
        val state = BoatState(Point2(2.0, 5.0), Point2(0.2, 0.5), 0.0, 0.0)
        val route = line(width = 9.0)
        controller.next(state, ice, route)
        val defaultSide = controller.decision!!.requestedSide
        controller.next(state, ice, route, IcerSettings(mapOf("free_feedback_width" to 1.0)))
        assertTrue(controller.decision!!.requestedSide > defaultSide)
        controller.next(state, ice, route, IcerSettings(mapOf("rate_deadzone" to 2.0)))
        assertEquals(0, controller.turnCommand(0.0, 0.0, 5.0, 0.0, 0.0))
        controller.next(state.copy(position = Point2.ZERO, velocity = Point2(0.0, 2.0)), ice,
            bend(radius = 3.0), IcerSettings(mapOf("reference_rate_limit" to 2.5)))
        assertEquals(-2.5, Math.toDegrees(controller.decision!!.referenceRate), 1e-9)
    }
}
