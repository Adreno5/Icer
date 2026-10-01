package adreno.turneler.navigation

import kotlin.math.PI
import kotlin.math.sin

/** The horizontal ice-boat tick used by Icer diagnostics; yaw and yaw rate mirror float arithmetic. */
object IcerPhysics {
    private val sine = FloatArray(65536) { index -> sin(index * (2.0 * PI / 65536.0)).toFloat() }
    private const val SINE_SCALE = 10430.378350470453

    fun facing(yawRadians: Double): Point2 {
        val index = (yawRadians.toFloat() * SINE_SCALE).toInt() and 65535
        return Point2(-sine[index].toDouble(), sine[(index + 16384) and 65535].toDouble())
    }

    fun damp(state: BoatState, friction: Double): BoatState {
        val retention = friction.coerceIn(0.05, 0.999)
        val omega = Math.toDegrees(state.angularVelocity).toFloat() * retention.toFloat()
        return state.copy(velocity = state.velocity * retention,
            angularVelocity = Math.toRadians(omega.toDouble()))
    }

    fun moveAfterDamping(state: BoatState, input: BoatInput): BoatState {
        var omega = Math.toDegrees(state.angularVelocity).toFloat()
        if (input.right) omega += 1.0f
        if (input.left) omega -= 1.0f
        val yaw = Math.toDegrees(state.yaw).toFloat() + omega
        var thrust = if (input.left != input.right && !input.forward && !input.backward) 0.005f else 0.0f
        if (input.forward) thrust += 0.04f
        if (input.backward) thrust -= 0.005f
        val acceleration = facing(Math.toRadians(yaw.toDouble())) * thrust.toDouble()
        val velocity = state.velocity + acceleration
        return BoatState(state.position + velocity, velocity,
            Math.toRadians(yaw.toDouble()), Math.toRadians(omega.toDouble()), acceleration, state.y)
    }

    fun step(state: BoatState, input: BoatInput, friction: Double) =
        moveAfterDamping(damp(state, friction), input)
}
