package adreno.turneler.navigation

import kotlin.math.*
import java.util.Locale
import adreno.turneler.navigation.IcerParameter.*

/** One measured, post-damping boat state produces one physical tick of WASD input. */
class IcerController {
    private var settings = IcerSettings.DEFAULT
    private var activeRoute: IcerRoute? = null
    private var progress = 0.0
    private var targetProgress = 0.0
    private var throttlePhase = 1.0
    private var controlTick = 0L

    var status = "scanning ice"
        private set
    var target: Point2? = null
        private set
    var targetTangent: Point2? = null
        private set
    var brakingDemand: Double? = null
        private set
    internal var decision: IcerDecision? = null
        private set

    fun reset() {
        activeRoute = null
        progress = 0.0
        targetProgress = 0.0
        throttlePhase = 1.0
        controlTick = 0L
        target = null
        targetTangent = null
        brakingDemand = null
        decision = null
        status = "scanning ice"
    }

    fun next(state: BoatState, environment: NavigationEnvironment, route: IcerRoute?,
        settings: IcerSettings = IcerSettings.DEFAULT, tick: Long = controlTick): BoatInput {
        this.settings = settings
        controlTick = tick + 1L
        decision = null
        if (route == null || route.length < settings[MIN_ROUTE]) {
            target = null
            targetTangent = null
            brakingDemand = null
            status = if (route == null) "no safe centre line" else "centre line too short"
            return stopYaw(state)
        }
        if (activeRoute !== route) {
            activeRoute = route
            progress = 0.0
            targetProgress = 0.0
        }
        val projectionResult = route.project(state.position, progress)
        val s = projectionResult.s
        progress = s
        if (!routeAheadSafe(route, environment, s, max(settings[SAFE_DISTANCE], state.speed * settings[SAFE_TICKS]))) {
            target = null
            targetTangent = null
            brakingDemand = null
            status = "route blocked; scanning ice"
            return stopYaw(state)
        }
        val projection = projectionResult.point
        val tangent = projectionResult.tangent.unit(state.forward)
        val normal = tangent.normal()
        val curvature = projectionResult.curvature
        val kappa = curvature.coerceIn(-1.0, 1.0)
        val parallel = state.velocity.dot(tangent)
        val lateral = state.velocity.dot(normal)
        val error = (state.position - projection).dot(normal)
        val h = route.halfWidth(s)
        val room = shortcutRoom(projection, normal, h, environment)
        val speed = state.speed

        val yawDistance = max(settings[YAW_LEAD] * max(0.0, parallel), IcerRoute.HULL_WIDTH)
        val paintedWidth = route.paintedHalfWidth
        val widthFactor = if (paintedWidth != null) {
            ((paintedWidth - IcerRoute.HULL_HALF_WIDTH) / IcerRoute.HULL_WIDTH).coerceIn(1.0, settings[PAINTED_WIDTH_FACTOR_MAX])
        } else 1.0
        val forwardDistance = max(yawDistance, settings[PAINTED_FORWARD_WIDTHS] * (paintedWidth ?: 0.0)) * widthFactor
        val signedSweep = if (paintedWidth != null) signedSweep(route, s, forwardDistance, tangent) else 0.0
        val leadWidths = settings[PAINTED_LEAD_WIDTHS] + settings[PAINTED_BEND_WIDTHS] * (abs(signedSweep) / PI).coerceIn(0.0, 1.0)
        val responseDistance = if (paintedWidth != null) {
            max(yawDistance, leadWidths * paintedWidth) * widthFactor
        } else yawDistance
        val angularSweep = angularSweep(route, s, responseDistance, tangent)
        val bend = max(abs(curvature) * responseDistance, angularSweep)
        val bendWeight = bend / (1.0 + bend)
        val inside = max(0.0, sign(kappa) * error)
        val clearance = max(0.0, h - IcerRoute.HULL_HALF_WIDTH)
        val request = min(route.length - s, responseDistance *
            (if (paintedWidth != null) bendWeight else sqrt(bendWeight)) *
            (1.0 + inside / (inside + clearance + 1e-9)))
        val preview = request
        var safe = 0.0
        if (preview > 1e-4) {
            if (safeLine(state.position, route.point(s + preview), environment)) {
                safe = preview
            } else {
                var low = 0.0
                var high = preview
                repeat(settings.int(PREVIEW_BISECTIONS)) {
                    val middle = (low + high) * 0.5
                    if (safeLine(state.position, route.point(s + middle), environment)) low = middle else high = middle
                }
                safe = low
            }
        }
        targetProgress = max(targetProgress, max(s, s + safe))
        val targetS = targetProgress.coerceAtMost(route.length)
        val targetPoint = route.point(targetS)
        target = targetPoint
        val targetTangent = route.tangent(targetS)
        this.targetTangent = targetTangent

        val friction = environment.friction(state.position).coerceIn(0.05, 0.999)
        val brake = brakingDemand(route, environment, s, speed, friction)
        brakingDemand = brake
        val edgeRoom = (settings[EDGE_WIDTH] - h).coerceIn(0.0, 1.0)
        val rearTurnWeight = brake * ((speed - settings[REAR_START]) / settings[REAR_BAND]).coerceIn(0.0, 1.0)
        val along = THRUST * (1.0 - 2.0 * brake)

        val requestedShortcut = settings[SHORTCUT_WEIGHT] * room * (targetPoint - projection).dot(normal)
        val shortcutSign = sign(requestedShortcut)
        var margin = Double.POSITIVE_INFINITY
        if (shortcutSign != 0.0) {
            val probeDistance = min(targetS - s, max(0.0, parallel) * settings[YAW_LEAD])
            for (i in 0..settings.int(SHORTCUT_SAMPLES)) {
                val nextS = (s + probeDistance * i / settings[SHORTCUT_SAMPLES]).coerceAtMost(route.length)
                val nextNormal = route.tangent(nextS).normal()
                if (nextNormal.dot(normal) <= 0.0) break
                margin = min(margin, iceMargin(route.point(nextS), nextNormal * shortcutSign, environment))
            }
        }
        val inward = max(0.0, lateral * shortcutSign)
        val stopping = inward * inward / (2.0 * THRUST)
        val shortcut = shortcutSign * min(abs(requestedShortcut), max(0.0, margin - stopping))
        val lateralError = error - shortcut
        val curveSpeed = max(0.0, parallel)
        val feedbackWidth = settings[if (paintedWidth != null) PAINTED_FEEDBACK_WIDTH else FREE_FEEDBACK_WIDTH]
        val tau = sqrt(2.0 * min(feedbackWidth, route.halfWidth(targetS)) / THRUST) +
            abs(Math.toDegrees(state.angularVelocity)) * settings[YAW_RESPONSE] * (1.0 - room)
        val kd = settings[LATERAL_DAMPING] + settings[NARROW_DAMPING] * (1.0 - room)
        val leadTicks = settings[YAW_LEAD]
        var steeringCurvature = kappa
        for (i in 1..settings.int(SWEEP_SAMPLES)) {
            val ahead = (s + curveSpeed * leadTicks * i / settings[SWEEP_SAMPLES]).coerceAtMost(route.length)
            steeringCurvature += route.curvature(ahead).coerceIn(-1.0, 1.0)
        }
        steeringCurvature /= settings[SWEEP_SAMPLES] + 1.0
        val previewDistance = targetS - s
        val previewCurvature = if (previewDistance > 1e-4) {
            tangent.angleTo(targetTangent) / previewDistance
        } else kappa
        val blend = room * previewDistance / (previewDistance + curveSpeed * leadTicks + 1e-9)
        steeringCurvature += blend * (previewCurvature - steeringCurvature)
        val positionGain = settings[POSITION_GAIN] + if (paintedWidth != null) 0.0 else
            settings[WIDE_POSITION_GAIN] * ((h - settings[WIDE_POSITION_START]) / settings[WIDE_POSITION_BAND]).coerceIn(0.0, 1.0)
        // Combine unsaturated curvature and feedback before applying the shared thrust circle.
        val requestedSide = curveSpeed * curveSpeed * steeringCurvature -
            positionGain * lateralError / (tau * tau) - kd * lateral / tau

        val scale = min(1.0, THRUST / hypot(along, requestedSide).coerceAtLeast(1e-9))
        val outward = max(0.0, lateral * sign(lateralError))
        val risk = ((abs(lateralError) + outward * tau) / max(0.25, clearance)).coerceIn(0.0, 1.0)
        val priority = max(1.0 - room, if (paintedWidth != null) 0.0 else risk)
        val steeringSide = (priority * requestedSide.coerceIn(-THRUST, THRUST) +
            (1.0 - priority) * requestedSide * scale) * (1.0 - settings[REAR_SIDE_REDUCTION] * rearTurnWeight)
        val requestedParallel = along * (priority + (1.0 - priority) * scale)

        val tangentYaw = atan2(-tangent.x, tangent.z)
        val previewYaw = atan2(-targetTangent.x, targetTangent.z)
        val guideYaw = tangentYaw + wrapped(previewYaw - tangentYaw) * room
        val forwardYaw = tangentYaw + asin((steeringSide / THRUST).coerceIn(-1.0, 1.0))
        val equilibrium = THRUST * friction / (1.0 - friction)
        val turnCost = settings[HEADING_COST] * THRUST * THRUST / (PI * PI) * (1.0 + speed / (speed + equilibrium))
        fun yawCost(yaw: Double): Double = turnCost * (
            wrapped(yaw - state.yaw).pow(2) +
                settings[GUIDE_WEIGHT] * (1.0 - rearTurnWeight) * wrapped(yaw - guideYaw).pow(2)
        )
        val range = sqrt(max(0.0, THRUST * THRUST - steeringSide * steeringSide))
        fun yawFor(parallelThrust: Double): Double =
            if (hypot(parallelThrust, steeringSide) < 1e-8) forwardYaw
            else tangentYaw + atan2(steeringSide, parallelThrust)
        fun cost(parallelThrust: Double): Double =
            (parallelThrust - requestedParallel).pow(2) + yawCost(yawFor(parallelThrust))
        var bestW = 0.0
        var bestCost = cost(bestW)
        for (candidate in listOf(-range, range)) {
            val candidateCost = cost(candidate)
            if (candidateCost < bestCost) {
                bestW = candidate
                bestCost = candidateCost
            }
        }
        for ((a, b) in listOf(-range to 0.0, 0.0 to range)) {
            var low = a
            var high = b
            repeat(settings.int(SOLVER_ITERATIONS)) {
                val left = high - (high - low) / GOLDEN
                val right = low + (high - low) / GOLDEN
                if (cost(left) < cost(right)) high = right else low = left
            }
            val candidate = (low + high) * 0.5
            val candidateCost = cost(candidate)
            if (candidateCost < bestCost) {
                bestW = candidate
                bestCost = candidateCost
            }
        }
        val bowAlong = state.forward.dot(tangent)
        val bowSide = state.forward.dot(normal)
        val coastCost = requestedParallel.pow(2) + steeringSide.pow(2) + yawCost(forwardYaw)
        val reverseCost = (-REVERSE * bowAlong - requestedParallel).pow(2) +
            (-REVERSE * bowSide - steeringSide).pow(2) + yawCost(forwardYaw)
        val useW = bestCost < min(coastCost, reverseCost)
        // Keep steering toward the best W heading during coast/S ticks as well.
        val desiredYaw = yawFor(bestW)

        // BoatPilot calls us at controlBoat HEAD, after vanilla has damped both velocity and
        // deltaRotation for this tick. Damping these measured values again would turn too late.
        val omegaRef = Math.toRadians(Math.toDegrees((kappa + steeringCurvature) * 0.5 * parallel)
            .coerceIn(-settings[REFERENCE_RATE_LIMIT], settings[REFERENCE_RATE_LIMIT]))
        val omegaDamped = state.angularVelocity
        val turn = turnCommand(Math.toDegrees(state.yaw), Math.toDegrees(omegaDamped),
            Math.toDegrees(desiredYaw), Math.toDegrees(omegaRef), edgeRoom)
        val nextOmega = Math.toDegrees(omegaDamped).toFloat() + turn.toFloat()
        val nextYaw = Math.toRadians((Math.toDegrees(state.yaw).toFloat() + nextOmega).toDouble())
        val plannedThrust = tangent * bestW + normal * steeringSide
        val nextBow = IcerPhysics.facing(nextYaw)
        val projected = nextBow.dot(plannedThrust)
        val turnDuty = cumulativeTurnDuty(state, friction, desiredYaw, omegaRef,
            edgeRoom, normal, steeringSide)
        val advancing = if (speed > 1e-9) max(0.0, nextBow.dot(state.velocity) / speed) else 0.0
        val cumulativeDuty = max(0.0, turnDuty - brake * advancing)
        val duty = if (useW) max(projected / THRUST, cumulativeDuty).coerceIn(0.0, 1.0) else 0.0
        var forward = false
        if (duty > 0.0) {
            throttlePhase += duty
            if (throttlePhase >= 1.0) {
                forward = true
                throttlePhase -= 1.0
            }
        }
        val yawError = abs(Math.toDegrees(wrapped(desiredYaw - state.yaw)))
        val coastBrake = !useW && reverseCost < coastCost && paintedWidth != null &&
            speed < settings[COAST_BRAKE_SPEED] && risk < settings[COAST_BRAKE_RISK] &&
            yawError < settings[COAST_BRAKE_ANGLE] && abs(Math.toDegrees(state.angularVelocity)) < settings[COAST_BRAKE_RATE]
        val reversePeriod = settings.int(if (speed <= settings[REVERSE_LOW_SPEED]) REVERSE_SLOW_PERIOD else REVERSE_FAST_PERIOD)
        val backward = !forward && !coastBrake && reverseCost < coastCost &&
            Math.floorMod(tick, reversePeriod.toLong()) != reversePeriod - 1L
        decision = IcerDecision(s, targetS, desiredYaw, omegaRef, steeringCurvature,
            requestedSide, steeringSide, requestedParallel, risk, useW, duty, coastBrake)
        status = String.format(Locale.ROOT, "centre %.1f b - brake %.2f - path %.0f b",
            abs(error), brake, route.length - s)
        return BoatInput(turn < 0, turn > 0, forward, backward)
    }

    /** Future bends request a thrust direction; no cruise speed is imposed on a straight. */
    private fun brakingDemand(route: IcerRoute, environment: NavigationEnvironment, s: Double, speed: Double,
        friction: Double): Double {
        if (speed < settings[BRAKE_MIN_SPEED]) return 0.0
        val drag = max(0.01, 1.0 - friction)
        val preview = min(route.length - s, max(speed * speed / (2.0 * THRUST), speed / drag) + settings[BRAKE_LEAD] * speed)
        val step = max(settings[BRAKE_STEP], speed * settings[BRAKE_SPEED_STEP])
        var required = 0.0
        var distance = step
        while (distance <= preview + step && preview > 0.0) {
            val d = distance.coerceAtMost(preview)
            val pointS = s + d
            val signedCurvature = route.curvature(pointS)
            val kappa = abs(signedCurvature)
            if (kappa >= 1e-6) {
                val width = route.halfWidth(pointS)
                val clearance = max(0.0, width - IcerRoute.HULL_HALF_WIDTH)
                var shortcutWeight = 0.0
                if (kappa * clearance > 1.0 && clearance > 1e-6) {
                    val before = route.point((pointS - width).coerceAtLeast(0.0))
                    val after = route.point((pointS + width).coerceAtMost(route.length))
                    if (safeLine(before, after, environment)) {
                        shortcutWeight = (kappa * clearance - 1.0).coerceIn(0.0, 1.0)
                    }
                }
                val effective = kappa + (1.0 / clearance.coerceAtLeast(1e-6) - kappa) * shortcutWeight
                val yawDistance = settings[YAW_LEAD] * sqrt(THRUST / effective)
                val before = route.curvature((pointS - yawDistance).coerceAtLeast(0.0))
                val after = route.curvature((pointS + yawDistance).coerceAtMost(route.length))
                val farAfter = route.curvature((pointS + 2.0 * yawDistance).coerceAtMost(route.length))
                val change = max(abs(signedCurvature - before), abs(after - signedCurvature))
                val rapidity = change / (effective + change) * (1.0 - shortcutWeight)
                val reversal = (abs(before) + abs(farAfter) - abs(before + farAfter)) /
                    (abs(before) + abs(farAfter) + 1e-9)
                val reversalCost = reversal * yawDistance / (yawDistance + 2.0 * width)
                val hullClearance = (clearance / width.coerceAtLeast(1e-9)).coerceIn(0.0, 1.0)
                val cornerRoom = (clearance / IcerRoute.HULL_WIDTH).coerceIn(settings[CORNER_ROOM_MIN], 1.0)
                val lateralBudget = THRUST * settings[LATERAL_BUDGET] *
                    (settings[HULL_BUDGET_BASE] + (1.0 - settings[HULL_BUDGET_BASE]) * hullClearance) * cornerRoom /
                    (1.0 + settings[RAPIDITY_WEIGHT] * rapidity + settings[REVERSAL_WEIGHT] * reversalCost)
                val excess = speed * speed * effective - lateralBudget
                if (excess > 0.0) {
                    val coastDistance = (speed - sqrt(lateralBudget / effective)) / drag
                    if (d <= coastDistance + settings[BRAKE_LEAD] * speed) {
                        val availableDistance = maxOf(settings[BRAKE_MIN_DISTANCE], settings[BRAKE_MIN_TICKS] * speed,
                            d - settings[BRAKE_LEAD] * speed)
                        required = max(required, excess / (2.0 * effective * availableDistance))
                    }
                }
            }
            if (d >= preview) break
            distance += step
        }
        return if (required > 0.0) ((THRUST + required - speed * (1.0 - friction)) / (2.0 * THRUST))
            .coerceIn(0.0, 1.0) else 0.0
    }

    /** Use each intermediate post-turn bow, not only the final heading, for W side impulse. */
    internal fun cumulativeTurnDuty(
        state: BoatState,
        friction: Double,
        desiredYaw: Double,
        omegaRef: Double,
        edgeRoom: Double,
        normal: Point2,
        steeringSide: Double,
    ): Double {
        if (abs(steeringSide) < 1e-9) return 0.0
        val direction = sign(steeringSide)
        val desiredDegrees = Math.toDegrees(desiredYaw)
        val referenceDegrees = Math.toDegrees(omegaRef)
        var yaw = Math.toDegrees(state.yaw).toFloat()
        var omega = Math.toDegrees(state.angularVelocity).toFloat()
        var available = 0.0
        var firstSide = 0.0
        var ticks = 0
        for (index in 0 until settings.int(FORECAST_TICKS)) {
            // The observed decision state is already damped at controlBoat HEAD.
            if (index > 0) omega *= friction.toFloat()
            val turn = turnCommand(yaw.toDouble(), omega.toDouble(),
                desiredDegrees, referenceDegrees, edgeRoom)
            omega += turn.toFloat()
            yaw += omega
            val bow = IcerPhysics.facing(Math.toRadians(yaw.toDouble()))
            val side = max(0.0, THRUST * direction * bow.dot(normal))
            if (index == 0) firstSide = side
            available += side
            ticks++
            if (ticks >= settings.int(FORECAST_MIN) && abs(Math.toDegrees(wrapped(Math.toRadians(desiredDegrees - yaw.toDouble())))) < settings[FORECAST_ANGLE]) break
        }
        return if (firstSide > 0.0 && available > 0.0) {
            (abs(steeringSide) * ticks / available).coerceIn(0.0, 1.0)
        } else 0.0
    }

    internal fun turnCommand(
        yawDegrees: Double,
        omegaDegrees: Double,
        desiredDegrees: Double,
        referenceDegrees: Double,
        edgeRoom: Double,
    ): Int {
        val yawError = Math.toDegrees(wrapped(Math.toRadians(desiredDegrees - yawDegrees)))
        val stoppingGain = settings[STOP_GAIN] * (1.0 + settings[EDGE_STOP_GAIN] * edgeRoom)
        val correction = sign(yawError) * min(abs(yawError) / (settings[YAW_RESPONSE_TICKS] + settings[EDGE_RESPONSE_TICKS] * edgeRoom),
            sqrt(2.0 * abs(yawError) / stoppingGain))
        val omegaError = referenceDegrees + correction - omegaDegrees
        return when {
            omegaError > settings[RATE_DEADZONE] -> 1
            omegaError < -settings[RATE_DEADZONE] -> -1
            else -> 0
        }
    }

    private fun shortcutRoom(point: Point2, normal: Point2, width: Double, environment: NavigationEnvironment): Double {
        if (width >= 2.0 * IcerRoute.HULL_WIDTH) return 1.0
        val margin = min(iceMargin(point, normal, environment), iceMargin(point, normal * -1.0, environment))
        return ((margin - IcerRoute.HULL_WIDTH) / IcerRoute.HULL_WIDTH).coerceIn(0.0, 1.0)
    }

    private fun angularSweep(route: IcerRoute, s: Double, distance: Double, initial: Point2): Double {
        var previous = initial
        var sweep = 0.0
        for (i in 1..settings.int(SWEEP_SAMPLES)) {
            val tangent = route.tangent((s + distance * i / settings[SWEEP_SAMPLES]).coerceAtMost(route.length))
            sweep += abs(previous.angleTo(tangent))
            previous = tangent
        }
        return sweep
    }

    private fun signedSweep(route: IcerRoute, s: Double, distance: Double, initial: Point2): Double {
        var previous = initial
        var sweep = 0.0
        for (i in 1..settings.int(SWEEP_SAMPLES)) {
            val tangent = route.tangent((s + distance * i / settings[SWEEP_SAMPLES]).coerceAtMost(route.length))
            sweep += previous.angleTo(tangent)
            previous = tangent
        }
        return sweep
    }

    private fun safeLine(from: Point2, to: Point2, environment: NavigationEnvironment): Boolean {
        val delta = to - from
        val count = ceil(delta.length() / 0.5).toInt().coerceAtLeast(1)
        if (!(0..count).all { environment.ice(from + delta * (it.toDouble() / count)) }) return false
        return environment.clear(from, to)
    }

    private fun routeAheadSafe(
        route: IcerRoute,
        environment: NavigationEnvironment,
        s: Double,
        distance: Double,
    ): Boolean {
        val end = min(route.length, s + distance)
        var previous = route.point(s)
        if (!environment.ice(previous)) return false
        var at = s + 0.25
        var sinceClear = 0.0
        while (at < end + 0.25) {
            val point = route.point(at.coerceAtMost(end))
            if (!environment.ice(point)) return false
            sinceClear += 0.25
            if (sinceClear >= 1.5 || at >= end) {
                if (!environment.clear(previous, point)) return false
                previous = point
                sinceClear = 0.0
            }
            if (at >= end) break
            at += 0.25
        }
        return true
    }

    private fun iceMargin(point: Point2, direction: Point2, environment: NavigationEnvironment): Double {
        var distance = 0.0
        while (distance + 0.25 <= settings[CONTROL_MARGIN] && environment.ice(point + direction * (distance + 0.25))) distance += 0.25
        return distance
    }

    private fun wrapped(angle: Double) = atan2(sin(angle), cos(angle))

    /** S suppresses the small forward shove that A/D alone adds while cancelling residual spin. */
    private fun stopYaw(state: BoatState): BoatInput {
        val rate = Math.toDegrees(state.angularVelocity)
        return when {
            rate > settings[STOP_DEADZONE] -> BoatInput(true, false, false, true)
            rate < -settings[STOP_DEADZONE] -> BoatInput(false, true, false, true)
            else -> BoatInput.RELEASED
        }
    }

    private companion object {
        const val THRUST = 0.04
        const val REVERSE = 0.005
        const val GOLDEN = 1.618033988749895
    }
}

/** Per-tick diagnostics; yaw and reference rate use the same radians as BoatState. */
internal data class IcerDecision(
    val progress: Double, val targetProgress: Double, val targetYaw: Double, val referenceRate: Double,
    val steeringCurvature: Double, val requestedSide: Double, val steeringSide: Double,
    val requestedParallel: Double, val edgeRisk: Double, val useW: Boolean, val duty: Double,
    val coastBrake: Boolean,
)
