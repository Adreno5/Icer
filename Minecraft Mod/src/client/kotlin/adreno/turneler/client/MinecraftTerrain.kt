package adreno.turneler.client

import adreno.turneler.navigation.NavigationEnvironment
import adreno.turneler.navigation.Point2
import net.minecraft.core.BlockPos
import net.minecraft.world.entity.Entity
import net.minecraft.world.entity.vehicle.boat.AbstractBoat
import net.minecraft.world.level.Level
import net.minecraft.world.level.block.Blocks
import net.minecraft.world.phys.AABB
import net.minecraft.world.phys.Vec3
import net.minecraft.world.phys.shapes.BooleanOp
import net.minecraft.world.phys.shapes.Shapes
import kotlin.math.*

class MinecraftTerrain(
    private val level: Level,
    private val boat: AbstractBoat,
    private val fixedOrigin: Point2? = null,
    private val fixedBox: AABB? = null,
) : NavigationEnvironment {
    /**
     * Where offsets are measured from. The box is captured once and only its shape is reused, but the
     * boat moves: a frozen origin measures clearance around the spot the boat was mounted at, which is
     * why the pilot could re-decide every tick for a whole ride.
     */
    private val origin: Point2 get() = fixedOrigin ?: Point2(boat.x, boat.z)
    private val box = fixedBox ?: boat.boundingBox

    /** Freeze the hull geometry for one worker decision while the live boat continues moving. */
    fun snapshot() = MinecraftTerrain(level, boat, Point2(boat.x, boat.z), boat.boundingBox)
    private val floorAllowance = box.ysize * 0.25
    private val frictionCache = HashMap<Point2, Double>()
    private val iceCache = HashMap<Long, Boolean>()

    /**
     * The collision box, moved to [position], has ice under all of it. This is the only ice question
     * the pilot asks, and it is asked on the box rather than on a block or an averaged field: the
     * footprint checks every touched block, so half a hull off the ice is not ice.
     */
    override fun ice(position: Point2) = hasIceUnder(boxAt(position))

    override fun iceBlock(
        x: Int,
        z: Int,
    ) = hasIce(x, z)

    override fun friction(position: Point2): Double {
        return frictionCache.getOrPut(position) {
            val local = boxAt(position)
            val contact = AABB(local.minX, local.minY - 0.001, local.minZ, local.maxX, local.minY, local.maxZ)
            val boatShape = Shapes.create(contact)
            var total = 0.0
            var count = 0
            val x0 = floor(contact.minX).toInt() - 1
            val x1 = ceil(contact.maxX).toInt() + 1
            val y0 = floor(contact.minY).toInt() - 1
            val y1 = ceil(contact.maxY).toInt() + 1
            val z0 = floor(contact.minZ).toInt() - 1
            val z1 = ceil(contact.maxZ).toInt() + 1
            for (bx in x0 until x1) {
                for (bz in z0 until z1) {
                    val edges = (if (bx == x0 || bx == x1 - 1) 1 else 0) + (if (bz == z0 || bz == z1 - 1) 1 else 0)
                    if (edges == 2) continue
                    for (by in y0 until y1) {
                        if (edges > 0 && (by == y0 || by == y1 - 1)) continue
                        val blockPos = BlockPos(bx, by, bz)
                        if (!level.hasChunkAt(blockPos)) continue
                        val state = level.getBlockState(blockPos)
                        if (Shapes.joinIsNotEmpty(
                                state.getCollisionShape(level, blockPos).move(bx.toDouble(), by.toDouble(), bz.toDouble()),
                                boatShape,
                                BooleanOp.AND,
                            )
                        ) {
                            total += state.block.friction
                            count++
                        }
                    }
                }
            }
            if (count == 0) 0.9 else (total / count).coerceIn(0.05, 0.999)
        }
    }

    override fun clear(
        from: Point2,
        to: Point2,
    ): Boolean {
        val delta = to - from
        val samples = ceil(delta.length() / 0.4).toInt().coerceAtLeast(1)
        for (i in 0..samples) {
            val point = from + delta * (i.toDouble() / samples)
            if (!level.hasChunkAt(BlockPos.containing(point.x, box.minY, point.z))) return false
            val local = boxAt(point)
            val support = AABB(local.minX, local.minY - 0.25, local.minZ, local.maxX, local.minY + floorAllowance, local.maxZ)
            if (level.getBlockCollisions(boat, support).none { !it.isEmpty }) return false
        }
        val local = boxAt(from)
        val hull = AABB(local.minX, local.minY + floorAllowance, local.minZ, local.maxX, local.maxY, local.maxZ)
        val movement = Vec3(delta.x, 0.0, delta.z)
        val collisions = level.getBlockCollisions(boat, hull.expandTowards(movement)).toList()
        if (collisions.any { shape -> shape.toAabbs().any { it.intersects(hull) } }) return false
        val resolved = Entity.collideBoundingBox(boat, movement, hull, level, collisions)
        return abs(resolved.x - movement.x) < 1e-4 && abs(resolved.z - movement.z) < 1e-4
    }

    /** Every block touched by the hull footprint must carry ice. */
    private fun hasIceUnder(local: AABB): Boolean {
        for (x in floor(local.minX).toInt()..floor(Math.nextDown(local.maxX)).toInt()) {
            for (z in floor(local.minZ).toInt()..floor(Math.nextDown(local.maxZ)).toInt()) {
                if (!hasIce(x, z)) return false
            }
        }
        return true
    }

    /**
     * One column of the world: it carries ice or it does not. Blue, packed and regular ice are all
     * drivable and all answer `true`; what they retain is friction, which stays where the dynamics
     * reads it. Cached per column, because a plan asks this thousands of times for the same map.
     */
    private fun hasIce(
        x: Int,
        z: Int,
    ): Boolean {
        val key = (x.toLong() shl 32) xor (z.toLong() and 0xffffffffL)
        return iceCache.getOrPut(key) {
            val top = floor(box.minY + floorAllowance).toInt()
            val bottom = floor(box.minY - 1.25).toInt()
            for (y in top downTo bottom) {
                val position = BlockPos(x, y, z)
                if (!level.hasChunkAt(position)) continue
                when (level.getBlockState(position).block) {
                    Blocks.BLUE_ICE, Blocks.PACKED_ICE, Blocks.ICE -> return@getOrPut true
                }
            }
            false
        }
    }

    private fun boxAt(point: Point2) = box.move(point.x - origin.x, 0.0, point.z - origin.z)

}
