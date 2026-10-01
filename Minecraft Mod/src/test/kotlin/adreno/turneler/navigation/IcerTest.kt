package adreno.turneler.navigation

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.*

class IcerTest {
    private fun terrain(ice: (Point2) -> Boolean) = object : NavigationEnvironment {
        override fun iceBlock(x: Int, z: Int) = ice(Point2(x + 0.5, z + 0.5))
        override fun ice(position: Point2) = ice(position)
        override fun clear(from: Point2, to: Point2) = true
        override fun friction(position: Point2) = 0.98
    }

    private fun hullSurface(
        blockIce: (Int, Int) -> Boolean,
        hullBlocked: (Point2) -> Boolean = { false },
    ) = object : NavigationEnvironment {
        override fun iceBlock(x: Int, z: Int) = blockIce(x, z)

        override fun ice(position: Point2): Boolean {
            val b = IcerRoute.HULL_HALF_WIDTH
            return (floor(position.x - b).toInt()..floor(Math.nextDown(position.x + b)).toInt()).all { x ->
                (floor(position.z - b).toInt()..floor(Math.nextDown(position.z + b)).toInt()).all { z ->
                    iceBlock(x, z)
                }
            }
        }

        override fun clear(from: Point2, to: Point2): Boolean {
            val delta = to - from
            val samples = ceil(delta.length() / 0.2).toInt().coerceAtLeast(1)
            return (0..samples).none { hullBlocked(from + delta * (it.toDouble() / samples)) }
        }

        override fun friction(position: Point2) = 0.98
    }

    private fun quarterIce(radius: Double = 18.0, margin: Double = 1.5) = terrain { p ->
        val before = hypot(p.x, p.z.coerceIn(0.0, 18.0) - p.z)
        val theta = atan2(p.z - 18.0, radius - p.x).coerceIn(0.0, PI / 2.0)
        val arc = Point2(radius - radius * cos(theta), 18.0 + radius * sin(theta))
        val curve = (p - arc).length()
        val after = hypot(p.x.coerceIn(radius, 70.0) - p.x, p.z - (18.0 + radius))
        min(before, min(curve, after)) <= margin
    }

    @Test fun liveTraceCentresAnOffsetBoatAndKeepsTheWholeSmoothedHullOnIce() {
        val ice = terrain { p -> p.z in -5.0..75.0 && abs(p.x - 3.0) <= 1.25 }
        val state = BoatState(Point2(2.0, 0.0), Point2(0.0, 0.4), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice, horizon = 48.0)!!
        assertTrue(route.length >= 40.0)
        assertTrue(route.path.points.all(ice::ice))
        val middle = (1..8).map { route.point(route.length * it / 9.0).x }
        assertTrue(middle.all { abs(it - 3.0) < 0.5 }, "centres=$middle")
        for (i in 0 until route.path.segments.lastIndex) {
            val end = route.path.segments[i].derivative(1.0).unit()
            val start = route.path.segments[i + 1].derivative(0.0).unit()
            assertTrue(end.dot(start) > 0.99, "cubic join $i must remain smooth")
        }
    }

    @Test fun localTraceFollowsAQuarterTurnInsteadOfDrivingStraightAcrossBareGround() {
        val ice = quarterIce()
        val state = BoatState(Point2(0.0, 0.0), Point2(0.0, 0.5), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice, horizon = 60.0)!!
        assertTrue(route.path.points.all(ice::ice))
        assertTrue(route.path.end.x > 10.0, "trace stopped before the bend: ${route.path.end}")
        assertTrue(route.path.end.z > 25.0)
    }

    @Test fun controllerBrakesExistingYawRateAndUsesTheMeasuredPostDampingState() {
        val ice = terrain { p -> abs(p.x) < 2.0 && p.z in -5.0..70.0 }
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.4), 0.0, Math.toRadians(7.0))
        val route = IcerPathGenerator().generate(state, ice, horizon = 48.0)!!
        val input = IcerController().next(state, ice, route)
        assertTrue(input.left, "positive yaw inertia should be counter-steered: $input")
        assertFalse(input.right)
    }

    @Test fun generatedQuarterTurnAndTickControllerKeepTheBoatOnIce() {
        val ice = quarterIce()
        var state = BoatState(Point2.ZERO, Point2(0.0, 0.5), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice, horizon = 60.0)!!
        val controller = IcerController()
        var reached = false
        var thrustWhileTurning = false
        for (tick in 0 until 110) {
            val measured = IcerPhysics.damp(state, ice.friction(state.position))
            val input = controller.next(measured, ice, route)
            thrustWhileTurning = thrustWhileTurning || (input.forward && (input.left || input.right))
            state = IcerPhysics.moveAfterDamping(measured, input)
            assertTrue(ice.ice(state.position), "tick $tick: ${state.position}, v=${state.velocity}, $input")
            if (state.position.x > 12.0 && state.position.z > 26.0) {
                reached = true
                break
            }
        }
        assertTrue(reached, "the boat never followed the bend: ${state.position}")
        assertTrue(thrustWhileTurning, "a drift turn needs W with A or D")
    }

    @Test fun controllerKeepsApplyingMeasuredFeedbackOnTheGeneratedStraightCentreLine() {
        val ice = terrain { p -> abs(p.x) < 1.5 && p.z in -5.0..70.0 }
        var state = BoatState(Point2(0.5, 0.0), Point2(0.0, 0.3), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice, horizon = 48.0)!!
        val controller = IcerController()
        repeat(35) { tick ->
            val measured = IcerPhysics.damp(state, ice.friction(state.position))
            val input = controller.next(measured, ice, route)
            state = IcerPhysics.moveAfterDamping(measured, input)
            assertTrue(ice.ice(state.position), "tick $tick: ${state.position}, $input")
        }
        assertTrue(state.position.z > 12.0, "controller did not progress: ${state.position}")
        assertTrue(abs(state.position.x) < 0.9, "controller did not recenter: ${state.position}")
    }

    @Test fun changedIceInvalidatesTheLiveRouteBeforeAnotherForwardPulse() {
        val ice = terrain { p -> abs(p.x) < 2.0 && p.z in -5.0..70.0 }
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.3), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice)!!
        val changed = terrain { p -> abs(p.x) < 2.0 && p.z < 3.0 }
        val controller = IcerController()
        assertEquals(BoatInput.RELEASED, controller.next(state, changed, route))
        assertTrue(controller.status.contains("blocked"))
    }

    @Test fun projectionPrefersPreviousProgressAtASelfCrossingAndFindsTheExactEnd() {
        fun line(a: Point2, b: Point2) = CubicBezier(a, a + (b - a) * (1.0 / 3.0),
            a + (b - a) * (2.0 / 3.0), b)
        val path = BezierPath(listOf(
            line(Point2(-4.0, -4.0), Point2(4.0, 4.0)),
            line(Point2(4.0, 4.0), Point2(-4.0, 4.0)),
            line(Point2(-4.0, 4.0), Point2(4.0, -4.0)),
        ))
        val profile = DoubleArray(50) { 3.0 }
        val friction = DoubleArray(50) { 0.98 }
        val route = IcerRoute(path, path.start, profile, friction)
        val first = route.project(Point2.ZERO, 0.0)
        val later = route.project(Point2.ZERO, route.length)
        assertTrue(first.s < path.length / 2.0)
        assertTrue(later.s > path.length / 2.0)
        assertEquals(route.length, route.project(path.end, route.length).s, 1e-8)
    }

    @Test fun refreshedCentreLineAgreesWithTheLineAlreadyUnderTheBoat() {
        val ice = quarterIce()
        val firstState = BoatState(Point2.ZERO, Point2(0.0, 0.5), 0.0, 0.0)
        val first = IcerPathGenerator().generate(firstState, ice, horizon = 60.0)!!
        val onLine = first.point(10.0)
        val heading = first.tangent(10.0)
        val state = BoatState(onLine, heading * 0.6, atan2(-heading.x, heading.z), 0.0)
        val refreshed = IcerPathGenerator().generate(state, ice, first, 60.0)!!
        val offset = refreshed.project(onLine).s
        for (ahead in 2..12 step 2) {
            val a = first.point((10.0 + ahead).coerceAtMost(first.length))
            val b = refreshed.point((offset + ahead).coerceAtMost(refreshed.length))
            assertTrue((a - b).length() < 1.5, "at $ahead blocks: $a vs $b")
        }
    }

    @Test fun aBlockCollisionTrimsTheLocalRouteBeforeTheWall() {
        val ice = terrain { p -> abs(p.x) < 2.0 && p.z in -5.0..70.0 }
        val wall = object : NavigationEnvironment by ice {
            override fun clear(from: Point2, to: Point2) = max(from.z, to.z) < 30.0
        }
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.4), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, wall, horizon = 48.0)!!
        assertTrue(route.length >= 20.0)
        assertTrue(route.path.end.z < 30.0)
    }

    @Test fun stationaryStartWithRepeatedReplansAcceleratesDownTheStraight() {
        val ice = terrain { p -> abs(p.x) < 1.5 && p.z in -5.0..100.0 }
        val generator = IcerPathGenerator()
        val controller = IcerController()
        var state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        var route: IcerRoute? = null
        for (tick in 0 until 80) {
            route = generator.generate(state, ice, route, 48.0)
            val measured = IcerPhysics.damp(state, ice.friction(state.position))
            val input = controller.next(measured, ice, route)
            state = IcerPhysics.moveAfterDamping(measured, input)
            assertTrue(ice.ice(state.position), "tick $tick: ${state.position}, $input")
        }
        assertTrue(state.position.z > 25.0, "stationary startup failed: ${state.position}")
        assertTrue(abs(state.position.x) < 1.0, "boat spun off centre: ${state.position}")
    }

    @Test fun stationaryQuarterTurnWithRepeatedReplansReachesTheBend() {
        val ice = quarterIce()
        val generator = IcerPathGenerator()
        val controller = IcerController()
        var state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        var route: IcerRoute? = null
        var reached = false
        for (tick in 0 until 140) {
            route = generator.generate(state, ice, route, 60.0)
            val measured = IcerPhysics.damp(state, ice.friction(state.position))
            val input = controller.next(measured, ice, route)
            state = IcerPhysics.moveAfterDamping(measured, input)
            assertTrue(ice.ice(state.position), "tick $tick: ${state.position}, v=${state.velocity}, $input")
            if (state.position.x > 10.0 && state.position.z > 25.0) {
                reached = true
                break
            }
        }
        assertTrue(reached, "stationary startup failed to turn: ${state.position}")
    }

    @Test fun narrowQuarterTurnStillYieldsAUsefulCentreLine() {
        val ice = quarterIce(radius = 8.0, margin = 1.2)
        val start = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        val generator = IcerPathGenerator()
        val route = generator.generate(start, ice, horizon = 48.0)
        assertNotNull(route, generator.status)
        assertTrue(route!!.path.end.x > 5.0, "stopped at ${route.path.end}")
        assertTrue(route.path.points.all(ice::ice))
    }

    @Test fun thinHullSafeWindingLineUsesShorterSweptSteps() {
        val ice = terrain { p -> abs(p.x - 2.0 * sin(p.z / 4.0)) <= 0.1 }
        val state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        val generator = IcerPathGenerator()
        val route = generator.generate(state, ice, horizon = 48.0)
        assertNotNull(route, generator.status)
        assertTrue(route!!.length > 40.0, "stopped after ${route.length} blocks")
        assertTrue(route.path.points.all(ice::ice), "the fitted curve left the narrow safe band")
    }

    @Test fun everyTouchedBlockMustBeIceIncludingThinDryMarkings() {
        val rail = hullSurface(blockIce = { x, z -> x == 0 && z in -10..80 })
        val state = BoatState(Point2(0.5, 0.5), Point2.ZERO, 0.0, 0.0)
        assertFalse(rail.ice(state.position))
        assertNull(IcerPathGenerator().generate(state, rail, horizon = 48.0))

        val marked = hullSurface(blockIce = { x, z -> x in -3..3 && z in -10..80 && !(x == 0 && z in 10..60) })
        assertFalse(marked.ice(Point2(0.5, 20.5)))
        val line = BezierPath(CubicBezier(state.position, Point2(0.5, 10.0),
            Point2(0.5, 20.0), Point2(0.5, 30.0)))
        val invalid = IcerRoute(line, state.position, DoubleArray(32) { 2.0 }, DoubleArray(32) { 0.98 })
        val controller = IcerController()
        assertEquals(BoatInput.RELEASED,
            controller.next(state.copy(position = Point2(0.5, 7.0)), marked, invalid))
        assertTrue(controller.status.contains("blocked"))

        val edge = hullSurface(blockIce = { x, z -> x <= 0 && z in -10..80 })
        assertFalse(edge.ice(Point2(1.5, 20.5)))
    }

    @Test fun quantizedWideIceProducesASmoothFastCentreLine() {
        val ice = terrain { p ->
            val band = floor((p.z + 2.0) / 3.0).toInt()
            val offset = when (Math.floorMod(band, 6)) {
                0, 1 -> 0.75
                2, 3 -> -0.75
                else -> 0.0
            }
            p.z in -5.0..90.0 && abs(p.x - offset) <= 4.5
        }
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.6), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice, horizon = 72.0)!!
        assertTrue(route.reachesHorizon)
        assertTrue(route.path.points.all(ice::ice))
        val maxCurvature = (16..(route.length.toInt() * 2 - 16)).maxOf { i ->
            abs(route.curvature(i * 0.5))
        }
        assertTrue(maxCurvature < 0.02, "quantized banks bent the route: $maxCurvature")
        val controller = IcerController()
        controller.next(state, ice, route)
        assertEquals(0.0, controller.brakingDemand!!, 1e-9)
    }

    @Test fun straightRoutesHaveNoEndpointSpeedCap() {
        val open = terrain { p -> abs(p.x) < 5.0 && p.z in -5.0..100.0 }
        val end = terrain { p -> abs(p.x) < 5.0 && p.z in -5.0..24.0 }
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.6), 0.0, 0.0)
        val generator = IcerPathGenerator()
        val openRoute = generator.generate(state, open, horizon = 48.0)!!
        // The terrain tracer can turn along the rink's end to find more ice. Use an actual
        // straight truncated route here so the test isolates endpoint speed behavior.
        val endPath = BezierPath(CubicBezier(Point2.ZERO, Point2(0.0, 7.0),
            Point2(0.0, 14.0), Point2(0.0, 21.0)))
        val endRoute = IcerRoute(endPath, Point2.ZERO, doubleArrayOf(5.0), doubleArrayOf(0.98))
        assertTrue(openRoute.reachesHorizon)
        assertFalse(endRoute.reachesHorizon)
        val openController = IcerController()
        val endController = IcerController()
        openController.next(state, open, openRoute)
        endController.next(state, end, endRoute)
        assertEquals(0.0, openController.brakingDemand!!, 1e-9)
        assertEquals(0.0, endController.brakingDemand!!, 1e-9)
    }

    @Test fun pvIsTheTangentAtTheCurrentPreviewPointAndClearsWithIt() {
        val ice = quarterIce()
        val state = BoatState(Point2.ZERO, Point2(0.0, 0.5), 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice, horizon = 60.0)!!
        val controller = IcerController()
        controller.next(state, ice, route)
        val p = controller.target!!
        val along = route.project(p).s
        assertTrue((controller.targetTangent!! - route.tangent(along)).length() < 1e-5)
        controller.next(state, ice, null)
        assertNull(controller.target)
        assertNull(controller.targetTangent)
    }

    @Test fun stationaryLaunchUsesForwardThrustInsteadOfTurningInPlace() {
        val ice = terrain { p -> abs(p.x) < 1.5 && p.z in -5.0..70.0 }
        val state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        val route = IcerPathGenerator().generate(state, ice)!!
        val first = IcerController().next(state, ice, route)
        assertTrue(first.forward, "a safe straight should launch with W: $first")
    }

    @Test fun sharpUpcomingBendRequestsReverseParallelThrustWithoutASpeedLimit() {
        val ice = terrain { true }
        val path = BezierPath(CubicBezier(Point2.ZERO, Point2(0.0, 12.0),
            Point2(8.0, 20.0), Point2(20.0, 20.0)))
        val route = IcerRoute(path, Point2.ZERO, DoubleArray(40) { 1.6 }, DoubleArray(40) { 0.98 })
        val controller = IcerController()
        controller.next(BoatState(Point2.ZERO, Point2(0.0, 1.5), 0.0, 0.0), ice, route)
        assertTrue(controller.brakingDemand!! > 0.5, "brake=${controller.brakingDemand}")
    }

    @Test fun turnForecastCountsIntermediatePostTurnForwardImpulses() {
        val controller = IcerController()
        val state = BoatState(Point2.ZERO, Point2.ZERO, Math.toRadians(20.0), 0.0)
        val duty = controller.cumulativeTurnDuty(state, 0.98, Math.toRadians(30.0), 0.0,
            0.0, Point2(-1.0, 0.0), 0.005)
        assertTrue(duty in 0.01..0.99, "turn duty $duty should pulse W during the turn")
        assertEquals(0.0, controller.cumulativeTurnDuty(state, 0.98, Math.toRadians(30.0), 0.0,
            0.0, Point2(1.0, 0.0), 0.005))
    }

    @Test fun iceTickDampsWorldVelocityThenTurnsBeforeThrust() {
        val state = BoatState(Point2.ZERO, Point2(1.0, 0.0), 0.0, 0.0)
        val next = IcerPhysics.step(state, BoatInput(false, true, true, false), 0.98)
        val bow = IcerPhysics.facing(Math.toRadians(1.0))
        assertEquals(Math.toRadians(1.0), next.yaw, 1e-8)
        assertEquals(0.98 + 0.04f * bow.x, next.velocity.x, 1e-8)
        assertEquals(0.04f * bow.z, next.velocity.z, 1e-8)
        assertEquals(next.velocity, next.position)
        assertTrue(next.velocity.x > 0.9, "turning the bow must not rotate old momentum")
    }

    @Test fun missingOrShortRouteStopsResidualSpinWithoutAddingForwardThrust() {
        val ice = terrain { true }
        var state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, Math.toRadians(5.0))
        val controller = IcerController()
        assertEquals(BoatInput(true, false, false, true), controller.next(state, ice, null))
        val shortPath = BezierPath(CubicBezier(Point2.ZERO, Point2(0.0, 0.5),
            Point2(0.0, 1.5), Point2(0.0, 2.0)))
        val short = IcerRoute(shortPath, Point2.ZERO, DoubleArray(3) { 2.0 }, DoubleArray(3) { 0.98 })
        assertEquals(BoatInput(true, false, false, true), controller.next(state, ice, short))
        repeat(12) {
            val measured = IcerPhysics.damp(state, 0.98)
            state = IcerPhysics.moveAfterDamping(measured, controller.next(measured, ice, null))
        }
        assertTrue(abs(Math.toDegrees(state.angularVelocity)) < 1.0, "spin remained ${state.angularVelocity}")
    }

    @Test fun tuningSnapshotsClampNonFiniteValuesAndPreserveIndependentDefaults() {
        val raw = mutableMapOf("beam" to 10000.0, "fine_step" to 3.0, "coarse_step" to 0.75,
            "smooth_min" to 0.8, "smooth_max" to 0.1, "lateral_damping" to Double.NaN)
        val settings = IcerSettings(raw)
        raw["beam"] = 4.0
        assertEquals(32, settings.int(IcerParameter.BEAM))
        assertEquals(0.75, settings[IcerParameter.FINE_STEP])
        assertEquals(0.1, settings[IcerParameter.SMOOTH_MIN])
        assertEquals(3.0, settings[IcerParameter.LATERAL_DAMPING])
        for (p in IcerParameter.entries) {
            assertEquals(p.default, IcerSettings.DEFAULT[p], "default for ${p.key}")
            assertEquals(p.default, IcerSettings(mapOf(p.key to Double.POSITIVE_INFINITY))[p])
            assertEquals(p.minimum, IcerSettings(mapOf(p.key to -1000.0))[p])
            assertTrue(settings[p].isFinite())
        }
    }

    @Test fun pathWidthAndStopYawTuningReachTheActualAlgorithms() {
        val ice = terrain { true }
        val state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        val generator = IcerPathGenerator()
        val narrowScan = generator.generate(state, ice,
            settings = IcerSettings(mapOf("width_limit" to 2.0)))!!
        val defaultScan = generator.generate(state, ice)!!
        assertEquals(IcerRoute.HULL_HALF_WIDTH + 2.0, narrowScan.halfWidth(10.0), 1e-9)
        assertTrue(defaultScan.halfWidth(10.0) > narrowScan.halfWidth(10.0))
        val rotating = state.copy(angularVelocity = Math.toRadians(1.0))
        val controller = IcerController()
        assertEquals(BoatInput(true, false, false, true), controller.next(rotating, ice, null))
        assertEquals(BoatInput.RELEASED, controller.next(rotating, ice, null,
            IcerSettings(mapOf("stop_deadzone" to 2.0))))
    }

    @Test fun vanillaTickOrderStillHoldsForEveryKeyCombinationOnBothIceFrictions() {
        for (friction in listOf(0.98, 0.989)) for (bits in 0..15) {
            val input = BoatInput(bits and 1 != 0, bits and 2 != 0, bits and 4 != 0, bits and 8 != 0)
            val state = BoatState(Point2(2.0, 3.0), Point2(0.3, 1.4), Math.toRadians(35.0), Math.toRadians(3.0))
            val next = IcerPhysics.step(state, input, friction)
            val omega = 3.0f * friction.toFloat() + (if (input.right) 1.0f else 0.0f) - (if (input.left) 1.0f else 0.0f)
            val yaw = 35.0f + omega
            var thrust = if (input.left != input.right && !input.forward && !input.backward) 0.005f else 0.0f
            if (input.forward) thrust += 0.04f
            if (input.backward) thrust -= 0.005f
            assertEquals(Math.toRadians(omega.toDouble()), next.angularVelocity, 1e-12)
            assertEquals(Math.toRadians(yaw.toDouble()), next.yaw, 1e-12)
            // The sine-table quantization is at most one entry; old momentum stays in world space.
            assertEquals(0.3 * friction - sin(next.yaw) * thrust, next.velocity.x, 5e-6)
            assertEquals(1.4 * friction + cos(next.yaw) * thrust, next.velocity.z, 5e-6)
            assertEquals(state.position + next.velocity, next.position)
        }
    }

    private fun straightRoute(x: Double, z: Double = 0.0, length: Double = 30.0): IcerRoute {
        val start = Point2(x, z)
        val path = BezierPath(CubicBezier(start, start + Point2(0.0, length / 3.0),
            start + Point2(0.0, length * 2.0 / 3.0), start + Point2(0.0, length)))
        return IcerRoute(path, start, doubleArrayOf(9.0), doubleArrayOf(0.989), reachesHorizon = true)
    }

    @Test fun twentyTickPointMeansRemoveAlternatingJitterWithoutPullingAMovingHorizonBack() {
        val ice = terrain { true }
        val history = IcerRouteHistory()
        var previous: IcerRoute? = null
        var maximumChange = 0.0
        for (tick in 0L..39L) {
            val raw = straightRoute(if (tick % 2L == 0L) -0.5 else 0.5, tick.toDouble())
            val averaged = IcerRouteAverager.average(raw, tick, history.snapshot(tick), ice)
            assertEquals(tick.toDouble(), averaged.path.start.z, 1e-9, "start at tick $tick")
            assertEquals(raw.path.end.z, averaged.path.end.z, 1e-9, "new tail at tick $tick")
            if (tick >= 19L) {
                assertEquals(0.0, averaged.point(8.0).x, 1e-9, "mean at tick $tick")
                previous?.let { maximumChange = max(maximumChange,
                    abs(averaged.point(8.0).x - it.point(8.0).x)) }
            }
            previous = averaged
            history.record(tick, raw)
        }
        assertTrue(maximumChange < 0.04, "raw route jumps 1 block; averaged jump=$maximumChange")
    }

    @Test fun historyUsesRealTickWeightsAndExpiresExactlyAtTheWindowBoundary() {
        val held = straightRoute(0.0)
        val fresh = straightRoute(1.0)
        val ice = terrain { true }
        val history = IcerRouteHistory()
        for (tick in 0L..19L) history.record(tick, held)
        val samples = history.snapshot(20L)
        assertEquals((1L..19L).toList(), samples.map { it.tick })
        assertEquals(0.05, IcerRouteAverager.average(fresh, 20L, samples, ice).point(8.0).x, 1e-9,
            "one new decision and nineteen held ticks must have a twenty-tick mean")
        assertSame(fresh, IcerRouteAverager.average(fresh, 39L, samples, ice), "tick 19 has expired at tick 39")
        val disabled = IcerSettings(mapOf("history_ticks" to 1.0))
        assertSame(fresh, IcerRouteAverager.average(fresh, 20L, samples, ice, disabled))
        history.clear()
        assertTrue(history.snapshot(20L).isEmpty())
    }

    @Test fun aFullTickWindowFreezesThePrefixWhileLaterPlansReplaceOnlyItsTail() {
        val ice = terrain { true }
        val settings = IcerSettings(mapOf("history_ticks" to 4.0))
        val held = straightRoute(0.0, length = 6.0)
        val fresh = straightRoute(1.0, length = 12.0)
        val history = (0L..2L).map { IcerRouteSample(it, held) }
        val warming = IcerRouteAverager.average(fresh, 2L, history, ice, settings)
        assertFalse(warming.frozenStart, "three contributing ticks cannot fill a four-tick window")
        val settled = IcerRouteAverager.average(fresh, 3L, history, ice, settings, warming)
        assertEquals(0.25, settled.path.start.x, 1e-9)
        assertEquals(6, settled.frozenSegments, "only points covered by all four ticks mature")
        assertTrue(settled.frozenLength < settled.length)

        // Old raw samples have expired, and both the fresh start and horizon sampling grid move.
        val moved = straightRoute(1.5, 2.25, 18.0)
        val updated = IcerRouteAverager.average(moved, 20L, history, ice, settings, settled)
        val projection = settled.project(moved.path.start)
        val retained = settled.frozenSuffix(projection.s)
        assertEquals(retained, updated.path.segments.take(retained.size), "stable controls must stay fixed")
        assertEquals(retained.size, updated.frozenSegments, "expired samples cannot mature the new tail")
        assertEquals(0.25, updated.path.start.x, 1e-9)
        assertEquals(moved.path.start.z, updated.path.start.z, 1e-9, "travelled prefix is trimmed")
        assertEquals(moved.path.end, updated.path.end, "the mutable tail follows the new horizon")
        val before = updated.path.segments[updated.frozenSegments - 1].derivative(1.0)
        val after = updated.path.segments[updated.frozenSegments].derivative(0.0)
        assertEquals(before.x, after.x, 1e-9)
        assertEquals(before.z, after.z, 1e-9, "tail joins without changing the frozen tangent")

        val revised = straightRoute(-1.0, 2.25, 20.0)
        val again = IcerRouteAverager.average(revised, 21L, emptyList(), ice, settings, updated)
        assertEquals(retained, again.path.segments.take(retained.size))
        assertEquals(revised.path.end, again.path.end)
        val disabled = IcerSettings(mapOf("history_ticks" to 1.0))
        assertSame(revised, IcerRouteAverager.average(revised, 21L, history, ice, disabled, again))
    }

    @Test fun repeatedSolveCountsAndMissingTickSamplesCannotPrematurelyFreezePoints() {
        val ice = terrain { true }
        val settings = IcerSettings(mapOf("history_ticks" to 4.0))
        val held = straightRoute(0.0)
        val fresh = straightRoute(1.0)
        val sparse = listOf(IcerRouteSample(0L, held), IcerRouteSample(2L, held), IcerRouteSample(2L, held))
        var route: IcerRoute? = null
        repeat(6) {
            val next = IcerRouteAverager.average(fresh, 3L, sparse, ice, settings, route)
            assertFalse(next.frozenStart, "only three distinct tick samples contributed")
            assertEquals(1.0 / 3.0, next.path.start.x, 1e-9)
            route = next
        }
    }

    @Test fun anUnsafeMeanTailFallsBackWithoutMovingTheSafeStablePrefix() {
        val ice = terrain { true }
        val settings = IcerSettings(mapOf("history_ticks" to 2.0))
        val short = straightRoute(2.0, length = 6.0)
        val stable = IcerRouteAverager.average(short, 1L, listOf(IcerRouteSample(0L, short)), ice, settings)
        val fresh = straightRoute(2.0, length = 12.0)
        val old = straightRoute(-2.0, length = 12.0)
        val marked = hullSurface(blockIce = { x, _ -> x != 0 })
        val updated = IcerRouteAverager.average(fresh, 3L, listOf(IcerRouteSample(2L, old)), marked, settings, stable)
        assertEquals(stable.path.segments, updated.path.segments.take(stable.frozenSegments))
        assertEquals(stable.frozenSegments, updated.frozenSegments)
        assertEquals(fresh.path.end, updated.path.end)
        assertTrue(updated.path.points.all(marked::ice))
    }

    @Test fun aMatureStartPointStaysFixedUntilTheBoatPassesIt() {
        val ice = terrain { true }
        val settings = IcerSettings(mapOf("history_ticks" to 2.0, "history_spacing" to 3.0))
        val old = straightRoute(0.0, length = 0.25)
        val fresh = straightRoute(1.0, length = 6.0)
        val stable = IcerRouteAverager.average(fresh, 1L, listOf(IcerRouteSample(0L, old)), ice, settings)
        assertTrue(stable.frozenStart)
        assertEquals(0, stable.frozenSegments, "only the start has a full window")
        val shifted = straightRoute(-1.0, length = 6.0)
        assertEquals(0.0, stable.project(shifted.path.start).s, 1e-9, "boat has not passed the fixed point")
        val updated = IcerRouteAverager.average(shifted, 10L, emptyList(), ice, settings, stable)
        assertEquals(stable.path.start, updated.path.start)
        assertTrue(updated.frozenStart)
        val moved = straightRoute(-1.0, 4.0, 6.0)
        assertSame(moved, IcerRouteAverager.average(moved, 11L, emptyList(), ice, settings, updated))
    }

    @Test fun laterMaturePointsStayFixedEvenWhenEarlierPointsHaveMissingContributions() {
        // An old route lost support near its start, but still supplies valid samples farther ahead.
        val ice = terrain { p -> p.x <= 1.5 || p.z >= 4.0 }
        val settings = IcerSettings(mapOf("history_ticks" to 2.0))
        val old = straightRoute(2.0, length = 12.0)
        val fresh = straightRoute(0.0, length = 12.0)
        val stable = IcerRouteAverager.average(fresh, 1L, listOf(IcerRouteSample(0L, old)), ice, settings)
        assertFalse(stable.frozenStart)
        assertEquals(0, stable.frozenSegments)
        assertTrue(stable.frozenTailPoints.isNotEmpty(), "farther points filled the window independently")
        assertTrue(stable.frozenTailPoints.all { abs(it.point.x - 1.0) < 1e-9 })

        val shifted = straightRoute(-1.0, length = 18.0)
        val updated = IcerRouteAverager.average(shifted, 20L, emptyList(), ice, settings, stable)
        val knots = updated.path.segments.map { it.p0 } + updated.path.end
        for (point in stable.frozenTailPoints) assertTrue(point.point in knots, "mature point moved: $point")
        assertEquals(shifted.path.end, updated.path.end, "extension remains fresh beyond mature points")
        assertFalse(updated.frozenStart)

        val revised = straightRoute(-0.5, length = 20.0)
        val filled = IcerRouteAverager.average(revised, 22L, listOf(IcerRouteSample(21L, fresh)), ice, settings, updated)
        assertTrue(filled.frozenStart)
        assertTrue(filled.frozenSegments > 0, "filling earlier points extends the continuously stable prefix")
        val filledKnots = filled.path.segments.map { it.p0 } + filled.path.end
        for (point in stable.frozenTailPoints) assertTrue(point.point in filledKnots)
    }

    @Test fun stableCurvesAreValidatedAndReprofiledAgainstCurrentTerrain() {
        val ice = terrain { true }
        val settings = IcerSettings(mapOf("history_ticks" to 2.0))
        val raw = straightRoute(0.0)
        val stable = IcerRouteAverager.average(raw, 1L, listOf(IcerRouteSample(0L, raw)), ice, settings)
        assertEquals(stable.path.segments.size, stable.frozenSegments)
        val changed = object : NavigationEnvironment by terrain({ p -> abs(p.x) < 2.0 }) {
            override fun friction(position: Point2) = 0.989
        }
        val reprofiling = IcerRouteAverager.average(raw, 3L, emptyList(), changed, settings, stable)
        assertEquals(stable.path.segments, reprofiling.path.segments)
        assertEquals(0.989, reprofiling.friction(8.0), 1e-12)
        assertTrue(reprofiling.halfWidth(8.0) < stable.halfWidth(8.0))

        val fresh = straightRoute(2.0)
        val dry = terrain { p -> abs(p.x) > 0.5 }
        assertSame(fresh, IcerRouteAverager.average(fresh, 4L, emptyList(), dry, settings, stable))
        val blocked = hullSurface(blockIce = { _, _ -> true }, hullBlocked = { p -> abs(p.x) < 0.2 })
        assertSame(fresh, IcerRouteAverager.average(fresh, 4L, emptyList(), blocked, settings, stable))
    }

    @Test fun averagingDoesNotClampOldEndpointsOrMixOppositeRouteDirections() {
        val ice = terrain { true }
        val old = straightRoute(-0.5, 0.0, 6.0)
        val fresh = straightRoute(0.5, 4.0, 30.0)
        val averaged = IcerRouteAverager.average(fresh, 1L, listOf(IcerRouteSample(0L, old)), ice)
        assertEquals(0.0, averaged.path.segments.first().p0.x, 1e-9)
        assertEquals(fresh.path.end, averaged.path.end, "old endpoint must not shorten the new horizon")
        assertEquals(0.5, averaged.path.segments[5].p0.x, 1e-9, "tail beyond old horizon is fresh")
        val backwards = straightRoute(-0.5, 34.0, -30.0)
        assertSame(fresh, IcerRouteAverager.average(fresh, 1L, listOf(IcerRouteSample(0L, backwards)), ice))
    }

    @Test fun aMeanAcrossADryMarkingOrWallFallsBackToTheFreshHullSafeRoute() {
        val raw = straightRoute(2.0)
        val old = straightRoute(-2.0)
        val marked = hullSurface(blockIce = { x, z -> x != 0 && z in -10..100 })
        assertTrue(marked.ice(raw.point(10.0)))
        assertTrue(marked.ice(old.point(10.0)))
        assertFalse(marked.ice(Point2(0.0, 10.0)))
        assertSame(raw, IcerRouteAverager.average(raw, 1L, listOf(IcerRouteSample(0L, old)), marked))
        val wall = hullSurface(blockIce = { _, _ -> true },
            hullBlocked = { p -> abs(p.x) < 0.2 && p.z in 10.0..20.0 })
        assertSame(raw, IcerRouteAverager.average(raw, 1L, listOf(IcerRouteSample(0L, old)), wall))
    }

    @Test fun averagedRoutesResampleTheirOwnWidthAndFrictionFromCurrentTerrain() {
        val ice = object : NavigationEnvironment by terrain({ p -> abs(p.x) < 5.0 }) {
            override fun friction(position: Point2) = if (position.x < 1.5) 0.98 else 0.989
        }
        val raw = straightRoute(2.0)
        val old = straightRoute(0.0)
        val averaged = IcerRouteAverager.average(raw, 1L, listOf(IcerRouteSample(0L, old)), ice)
        assertEquals(1.0, averaged.point(8.0).x, 1e-9)
        assertEquals(0.98, averaged.friction(8.0), 1e-12)
        assertTrue(averaged.halfWidth(8.0) < raw.halfWidth(8.0))
    }

    @Test fun twentyTickAveragingAndTickFeedbackKeepTheRepeatedQuarterTurnRideOnIce() {
        val ice = quarterIce()
        val generator = IcerPathGenerator()
        val controller = IcerController()
        val history = IcerRouteHistory()
        var state = BoatState(Point2.ZERO, Point2.ZERO, 0.0, 0.0)
        var route: IcerRoute? = null
        var reached = false
        for (tick in 0L until 140L) {
            val measured = IcerPhysics.damp(state, ice.friction(state.position))
            val raw = generator.generate(measured, ice, route, 60.0)
            route = raw?.let { IcerRouteAverager.average(it, tick, history.snapshot(tick), ice, incumbent = route) }
            history.record(tick, raw)
            val input = controller.next(measured, ice, route)
            state = IcerPhysics.moveAfterDamping(measured, input)
            assertTrue(ice.ice(state.position), "tick $tick: ${state.position}, v=${state.velocity}, $input")
            if (state.position.x > 10.0 && state.position.z > 25.0) { reached = true; break }
        }
        assertTrue(reached, "averaged ride did not reach the bend: ${state.position}")
    }
}
