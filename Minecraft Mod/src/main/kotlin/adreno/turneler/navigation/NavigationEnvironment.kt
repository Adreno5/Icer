package adreno.turneler.navigation

interface NavigationEnvironment {
    /**
     * The one column of the world at these block coordinates: it carries ice, or it does not. Blue,
     * packed and regular ice are all ice; what they retain is friction, which only the dynamics reads.
     *
     * This replaced a graded surface weight - blue 1.0, packed 0.8, regular 0.5 - averaged over a disk
     * around a point, and that average is exactly how a route came to be planned a hull-width into the
     * edge: a spot can score 0.4 on it, blue ice next to bare ground, and still have nothing under half
     * the collision box.
     */
    fun iceBlock(
        x: Int,
        z: Int,
    ): Boolean

    /**
     * The collision box, centred on [position], is standing on ice. An adapter implements this on its
     * own box: `MinecraftTerrain` checks every block touched by the hull footprint.
     *
     * Both ice questions are abstract on purpose. A delegating adapter (`by someEnvironment`) that
     * overrides only one of them still answers the other from its delegate, so a fixture that meant
     * "no ice anywhere" silently answered "ice everywhere" - a trap worth writing two lines for.
     */
    fun ice(position: Point2): Boolean

    fun clear(
        from: Point2,
        to: Point2,
    ): Boolean

    fun friction(position: Point2): Double

}

data class BoatState(
    val position: Point2,
    val velocity: Point2,
    val yaw: Double,
    val angularVelocity: Double,
    val acceleration: Point2 = Point2.ZERO,
    val y: Double = 0.0,
) {
    val speed get() = velocity.length()
    val forward get() = Point2.facing(yaw)
    val travel get() = velocity.unit(forward)
}
