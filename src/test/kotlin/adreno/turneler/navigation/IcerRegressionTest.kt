package adreno.turneler.navigation

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import java.nio.file.Files
import java.nio.file.Path
import kotlin.math.*

/** Discrete keys on tile ice; report both key shares and per-tick duties, including drift safety. */
class IcerRegressionTest {
    @Test fun representativeTileTracksFinishOnBothIceFrictions() {
        val results = IcerRegression.measure { route, environment ->
            val controller = IcerController()
            val drive: (BoatState, Long) -> BoatInput =
                { state, tick -> controller.next(state, environment, route, tick = tick) }
            drive
        }
        IcerRegression.write("icer-regression.csv", results)
        for (result in results) {
            assertTrue(result.reached, "$result")
            assertEquals(0, result.offIce, "$result")
            assertEquals(0, result.centreOffIce, "$result")
            assertTrue(result.maxError < result.trackWidth, "$result")
            assertTrue(result.longestS < 10, "$result")
        }
        val keyTotal = results.sumOf { it.w + it.a + it.s + it.d }.toDouble()
        assertTrue(results.sumOf { it.w } / keyTotal > 0.6, "aggregate W share")
        assertTrue(results.sumOf { it.s } / keyTotal < 0.2, "aggregate S share")
    }
}

internal object IcerRegression {
    data class Result(
        val name: String, val friction: Double, val reached: Boolean, val ticks: Int, val trackWidth: Double,
        val w: Int, val a: Int, val s: Int, val d: Int, val longestS: Int,
        val offIce: Int, val centreOffIce: Int, val maxError: Double, val rmsError: Double,
        val totalYaw: Double, val turnChanges: Int, val adjacentCounterTurns: Int,
    ) {
        private val keyTotal get() = (w + a + s + d).coerceAtLeast(1).toDouble()
        val wShare get() = w / keyTotal
        val sShare get() = s / keyTotal
        val wDuty get() = w.toDouble() / ticks
        val sDuty get() = s.toDouble() / ticks
        fun csv() = listOf(name, friction, reached, ticks, w, a, s, d, wShare, sShare, wDuty, sDuty,
            longestS, offIce, centreOffIce, maxError, rmsError, totalYaw, turnChanges, adjacentCounterTurns).joinToString(",")
    }

    private fun line(a: Point2, b: Point2) = CubicBezier(a, a + (b - a) * (1.0 / 3.0),
        a + (b - a) * (2.0 / 3.0), b)

    fun measure(factory: (IcerRoute, NavigationEnvironment) -> (BoatState, Long) -> BoatInput): List<Result> {
        val handle = 18.0 * 4.0 / 3.0 * tan(PI / 8.0)
        val tracks = listOf(
            Triple("narrow-straight", BezierPath(line(Point2.ZERO, Point2(0.0, 100.0))), 1.5),
            Triple("quarter-turn", BezierPath(listOf(
                line(Point2.ZERO, Point2(0.0, 18.0)),
                CubicBezier(Point2(0.0, 18.0), Point2(0.0, 18.0 + handle),
                    Point2(18.0 - handle, 36.0), Point2(18.0, 36.0)),
                line(Point2(18.0, 36.0), Point2(60.0, 36.0)),
            )), 3.0),
            Triple("alternating-bends", BezierPath(listOf(
                line(Point2.ZERO, Point2(0.0, 12.0)),
                CubicBezier(Point2(0.0, 12.0), Point2(0.0, 27.0), Point2(18.0, 27.0), Point2(18.0, 42.0)),
                CubicBezier(Point2(18.0, 42.0), Point2(18.0, 57.0), Point2(-8.0, 57.0), Point2(-8.0, 72.0)),
                line(Point2(-8.0, 72.0), Point2(-8.0, 105.0)),
            )), 5.0),
        )
        return tracks.flatMap { (name, path, width) ->
            val geometry = IcerRoute(path, path.start, doubleArrayOf(width), doubleArrayOf(0.98))
            // The finish plane is a checkpoint on continuing ice, not the physical ice edge.
            // This controller has no endpoint speed cap and crossing it need not stop the boat.
            val samples = (0..ceil(geometry.length / 0.25).toInt()).map { geometry.point(it * 0.25) } +
                (1..16).flatMap { i -> listOf(path.start - geometry.tangent(0.0) * (i * 0.25),
                    path.end + geometry.tangent(geometry.length) * (i * 0.25)) }
            listOf(0.98, 0.989).map { friction ->
                val environment = object : NavigationEnvironment {
                    override fun iceBlock(x: Int, z: Int): Boolean {
                        val centre = Point2(x + 0.5, z + 0.5)
                        return samples.any { (centre - it).lengthSquared() <= width * width }
                    }
                    override fun ice(position: Point2): Boolean {
                        val half = IcerRoute.HULL_HALF_WIDTH
                        return (floor(position.x - half).toInt()..floor(Math.nextDown(position.x + half)).toInt()).all { x ->
                            (floor(position.z - half).toInt()..floor(Math.nextDown(position.z + half)).toInt()).all { z ->
                                iceBlock(x, z)
                            }
                        }
                    }
                    override fun clear(from: Point2, to: Point2) = true
                    override fun friction(position: Point2) = friction
                }
                val route = IcerRoute.profiled(path, path.start, environment, 8.0, false)
                val drive = factory(route, environment)
                var state = BoatState(path.start, Point2.ZERO, atan2(-route.tangent(0.0).x, route.tangent(0.0).z), 0.0)
                var ticks = 0
                var reached = false
                var w = 0; var a = 0; var s = 0; var d = 0
                var runS = 0; var longestS = 0; var offIce = 0; var centreOffIce = 0
                var maxError = 0.0; var squaredError = 0.0; var totalYaw = 0.0
                var lastTurn = 0; var previousTurn = 0; var changes = 0; var adjacent = 0
                var progress = 0.0
                for (tick in 0 until 500) {
                    val measured = IcerPhysics.damp(state, environment.friction(state.position))
                    val input = drive(measured, tick.toLong())
                    if (input.forward) w++
                    if (input.left) a++
                    if (input.backward) s++
                    if (input.right) d++
                    runS = if (input.backward) runS + 1 else 0
                    longestS = max(longestS, runS)
                    val turn = (if (input.right) 1 else 0) - (if (input.left) 1 else 0)
                    if (turn != 0) {
                        if (lastTurn != 0 && lastTurn != turn) changes++
                        if (previousTurn == -turn) adjacent++
                        lastTurn = turn
                    }
                    previousTurn = turn
                    val moved = IcerPhysics.moveAfterDamping(measured, input)
                    totalYaw += abs(Math.toDegrees(atan2(sin(moved.yaw - state.yaw), cos(moved.yaw - state.yaw))))
                    state = moved
                    val projection = route.project(state.position, progress)
                    progress = projection.s
                    val lateralError = (state.position - projection.point).dot(projection.tangent.normal())
                    maxError = max(maxError, abs(lateralError))
                    squaredError += lateralError * lateralError
                    if (!environment.ice(state.position)) offIce++
                    if (!environment.iceBlock(floor(state.position.x).toInt(), floor(state.position.z).toInt())) centreOffIce++
                    ticks++
                    val endOffset = state.position - path.end
                    if (progress >= route.length - 1.0 && endOffset.length() < width &&
                        endOffset.dot(route.tangent(route.length)) >= 0.0) {
                        reached = true
                        break
                    }
                    if (offIce > 0) break
                }
                Result(name, friction, reached, ticks, width, w, a, s, d, longestS, offIce, centreOffIce,
                    maxError, sqrt(squaredError / ticks), totalYaw, changes, adjacent)
            }
        }
    }

    fun write(filename: String, results: List<Result>) {
        Files.createDirectories(Path.of("build"))
        val header = "track,friction,reached,ticks,W,A,S,D,W_share,S_share,W_duty,S_duty,longest_S,off_ice,centre_off_ice,max_lateral_error,rms_lateral_error,total_yaw,AD_changes,adjacent_counter_turns"
        Files.writeString(Path.of("build", filename), header + "\n" + results.joinToString("\n") { it.csv() } + "\n")
    }
}
