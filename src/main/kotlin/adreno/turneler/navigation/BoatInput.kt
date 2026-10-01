package adreno.turneler.navigation

/** Vanilla key state for one tick. A/D are signed angular acceleration, not a steering angle. */
data class BoatInput(
    val left: Boolean,
    val right: Boolean,
    val forward: Boolean,
    val backward: Boolean,
) {
    companion object {
        val RELEASED = BoatInput(false, false, false, false)
    }
}

