package adreno.turneler.client

import adreno.turneler.navigation.*
import net.minecraft.client.Minecraft
import net.minecraft.gizmos.Gizmo
import net.minecraft.gizmos.GizmoStyle
import net.minecraft.gizmos.Gizmos
import net.minecraft.util.ARGB
import net.minecraft.world.entity.vehicle.boat.AbstractBoat
import net.minecraft.world.phys.Vec3
import org.slf4j.LoggerFactory
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.Future
import kotlin.math.max

/** Icer is the sole pilot: the worker refreshes the line, measured feedback chooses keys each tick. */
class BoatPilot(private val config: TurnelerConfig) {
    private val logger = LoggerFactory.getLogger(BoatPilot::class.java)
    private val icerGenerator = IcerPathGenerator()
    private val icerController = IcerController()
    private var icerRoute: IcerRoute? = null
    private var rawRoute: IcerRoute? = null
    private val routeHistory = IcerRouteHistory()
    private var routeTick = 0L
    private data class RouteResult(val raw: IcerRoute, val averaged: IcerRoute)
    private var icerPending: Future<RouteResult?>? = null
    private var icerPlanStatus = "scanning ice"
    private var icerInput = BoatInput.RELEASED
    private var lastIcerErrorLog = 0L
    private var controlledBoat: AbstractBoat? = null
    private var lastVelocity = Point2.ZERO
    private var previousPosition: Point2? = null
    private var renderY = 0.0
    private var settings = IcerSettings(config.parameters)
    private var settingsSource = config.parameters
    private var horizon = config.icerHorizon
    private val plannerExecutor = Executors.newSingleThreadExecutor { task ->
        Thread(task, "turneler-icer-planner").apply { isDaemon = true }
    }

    val icerStatus: String get() = if (icerRoute == null) icerPlanStatus else icerController.status
    val status: String get() = when {
        !config.enabled -> "自动驾驶已关闭"
        controlledBoat == null -> "乘坐冰船后开始"
        icerRoute != null -> "正在跟踪冰面路线"
        icerPlanStatus.startsWith("planner error") -> "路线扫描失败，正在重试"
        else -> "正在寻找安全路线"
    }
    val hudLines: List<String> get() = listOf(
        "Turneler",
        status,
        String.format(Locale.ROOT, "速度  %.2f 格/tick", lastVelocity.length()),
        "按键  $keys",
    )
    private val keys: String get() = buildString {
        if (icerInput.forward) append('W')
        if (icerInput.left) append('A')
        if (icerInput.backward) append('S')
        if (icerInput.right) append('D')
        if (isEmpty()) append("松开")
    }

    /** Vanilla has already damped velocity and yaw rate when this hook runs. */
    fun controlBoat(client: Minecraft, boat: AbstractBoat, angularVelocity: Float) {
        val player = client.player ?: return
        val level = client.level ?: return
        if (boat.controllingPassenger !== player || boat.level() !== level) return
        val manual = player.input.keyPresses
        boat.setInput(manual.left(), manual.right(), manual.forward(), manual.backward())
        if (!config.enabled) {
            reset()
            return
        }
        if (settingsSource != config.parameters || horizon != config.icerHorizon) {
            settingsSource = config.parameters
            settings = IcerSettings(config.parameters)
            horizon = config.icerHorizon
            reset()
        }
        if (controlledBoat !== boat) {
            reset()
            controlledBoat = boat
        }
        val position = Point2(boat.x, boat.z)
        val velocity = Point2(boat.deltaMovement.x, boat.deltaMovement.z)
        val worldTerrain = MinecraftTerrain(level, boat)
        // Losing full ice support asks Icer for a way back. Only blocked motion stops the pilot.
        if (!worldTerrain.clear(position, position)) {
            reset()
            return
        }
        val teleported = previousPosition?.let {
            (position - it - lastVelocity).length() > max(4.0, velocity.length() * 3)
        } ?: false
        if (teleported) {
            reset()
            controlledBoat = boat
        }
        val state = BoatState(
            position, velocity, Math.toRadians(boat.yRot.toDouble()),
            Math.toRadians(angularVelocity.toDouble()),
            if (previousPosition == null) Point2.ZERO else velocity - lastVelocity, boat.y,
        )
        val tick = routeTick++
        icerPending?.let { future ->
            if (future.isDone) {
                val outcome = runCatching { future.get() }
                val candidate = outcome.getOrNull()
                outcome.exceptionOrNull()?.let { failure ->
                    val cause = failure.cause ?: failure
                    icerPlanStatus = "planner error: ${cause.javaClass.simpleName}"
                    val now = System.currentTimeMillis()
                    if (now - lastIcerErrorLog >= 5000) {
                        logger.warn("Icer centre-line generation failed", cause)
                        lastIcerErrorLog = now
                    }
                }
                if (candidate != null &&
                    (candidate.averaged.origin - state.position).length() <= 8.0 &&
                    candidate.averaged.project(state.position).distance <= 4.0
                ) {
                    rawRoute = candidate.raw
                    icerRoute = candidate.averaged
                    icerPlanStatus = icerGenerator.status
                } else if (candidate == null && outcome.isSuccess) {
                    icerPlanStatus = icerGenerator.status
                } else if (candidate != null) {
                    icerPlanStatus = "discarded stale centre line"
                }
                icerPending = null
            }
        }
        routeHistory.record(tick, rawRoute, settings.int(IcerParameter.HISTORY_TICKS))
        if (icerPending == null) {
            val capturedState = state
            val capturedTerrain = worldTerrain.snapshot()
            val incumbent = icerRoute
            val horizon = config.icerHorizon.toDouble()
            val capturedSettings = settings
            val capturedHistory = routeHistory.snapshot(tick, settings.int(IcerParameter.HISTORY_TICKS))
            icerPending = plannerExecutor.submit<RouteResult?> {
                icerGenerator.generate(capturedState, capturedTerrain, incumbent, horizon, capturedSettings)?.let { raw ->
                    RouteResult(raw, IcerRouteAverager.average(raw, tick, capturedHistory, capturedTerrain,
                        capturedSettings, incumbent))
                }
            }
        }
        val input = icerController.next(state, worldTerrain, icerRoute, settings, tick)
        icerInput = input
        boat.setInput(input.left, input.right, input.forward, input.backward)
        lastVelocity = velocity
        previousPosition = position
        renderY = boat.boundingBox.minY + boat.bbHeight + 0.12
    }

    fun tick(client: Minecraft) {
        if (!config.enabled || client.player?.vehicle !== controlledBoat || client.level !== controlledBoat?.level()) {
            reset()
        }
    }

    fun renderGizmos() {
        if (!config.enabled || !config.showPrediction) return
        icerRoute?.let { route ->
            val samples = kotlin.math.ceil(route.length).toInt().coerceAtLeast(1)
            for (i in 1..samples) {
                val a = route.point(route.length * (i - 1) / samples)
                val b = route.point(route.length * i / samples)
                val line = Gizmos.line(worldPoint(a), worldPoint(b), 0xFF39FF14.toInt(), 3.0f)
                if (config.alwaysOnTop) line.setAlwaysOnTop()
            }
        }
        val p = icerController.target
        val tangent = icerController.targetTangent
        if (p != null && tangent != null) {
            val pvColor = 0xFFFFB347.toInt()
            val origin = worldPoint(p).add(0.0, 0.04, 0.0)
            val tip = worldPoint(p + tangent * 2.5).add(0.0, 0.04, 0.0)
            val base = p + tangent * 1.95
            val left = worldPoint(base + tangent.normal() * 0.32).add(0.0, 0.04, 0.0)
            val right = worldPoint(base - tangent.normal() * 0.32).add(0.0, 0.04, 0.0)
            val shaft = Gizmos.line(origin, tip, pvColor, 3.0f)
            val head = Gizmos.addGizmo(Gizmo { primitives, alpha ->
                primitives.addTriangleFan(arrayOf(tip, left, right), ARGB.multiplyAlpha(pvColor, alpha))
            })
            // A filled horizontal circle stays round in world space and is larger than the
            // old point primitive; draw it after the shaft so P remains unambiguous.
            val point = Gizmos.circle(origin, 0.32f, GizmoStyle.fill(0xFFFFFFFF.toInt()))
            if (config.alwaysOnTop) {
                shaft.setAlwaysOnTop()
                head.setAlwaysOnTop()
                point.setAlwaysOnTop()
            }
        }
    }

    private fun worldPoint(point: Point2) = Vec3(point.x, renderY, point.z)

    private fun reset() {
        icerPending?.cancel(true)
        icerPending = null
        icerRoute = null
        rawRoute = null
        routeHistory.clear()
        routeTick = 0L
        icerController.reset()
        icerPlanStatus = "scanning ice"
        icerInput = BoatInput.RELEASED
        controlledBoat = null
        lastVelocity = Point2.ZERO
        previousPosition = null
    }
}
