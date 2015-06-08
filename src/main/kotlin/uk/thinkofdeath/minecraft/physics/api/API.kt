package uk.thinkofdeath.minecraft.physics.api

import com.badlogic.gdx.math.Vector3
import org.bukkit.Location
import org.bukkit.block.BlockState
import org.bukkit.inventory.ItemStack
import org.bukkit.material.MaterialData
import org.bukkit.plugin.Plugin
import org.bukkit.util.Vector
import uk.thinkofdeath.minecraft.physics.PBlock
import uk.thinkofdeath.minecraft.physics.PhysicsPlugin

class PhysicsAPI(val owner: Plugin, private val plugin: PhysicsPlugin) {

    /**
     * Controls whether explosions are tracked and replaced
     * with physics blocks
     */
    public fun setExplosionTracking(track: Boolean) {
        plugin.trackExplosions = track
    }

    /**
     * Spawns a block at the location copying its type and drops
     * from the passed block state.
     */
    public fun spawnBlock(loc: Location, block: BlockState): PhysicsBlock {
        val bl = PBlock(plugin, loc, block)
        plugin.blocks.add(bl)
        return PhysicsBlock(bl, this)
    }

    /**
     * Spawns a block at the location with no drops
     */
    public fun spawnBlock(loc: Location, data: MaterialData): PhysicsBlock {
        val bl = PBlock(plugin, loc, data.toItemStack())
        plugin.blocks.add(bl)
        return PhysicsBlock(bl, this)
    }

    /**
     * Spawns a block at the location
     */
    public fun spawnBlock(loc: Location, data: MaterialData, drops: Collection<ItemStack>): PhysicsBlock {
        val bl = PBlock(plugin, loc, data.toItemStack(), drops)
        plugin.blocks.add(bl)
        return PhysicsBlock(bl, this)
    }
}

class PhysicsBlock(private val block: PBlock, private val api: PhysicsAPI) {

    /**
     * Returns the current location of the block
     */
    public fun getLocation(): Location {
        return block.location.clone()
    }

    /**
     * Changes the location of the block. Cannot move between worlds
     */
    public fun setLocation(loc: Location) {
        if (loc.getWorld() != block.location.getWorld()) {
            throw IllegalArgumentException(
                    "Block Physics: Plugin %s tried to move a block to a different world".format(api.owner.getName())
            )
        }
        block.location.setX(loc.getX())
        block.location.setY(loc.getY())
        block.location.setZ(loc.getZ())

        block.transform.idt()
        block.transform.setTranslation(
                block.location.getX().toFloat(),
                block.location.getY().toFloat() + 1.8f,
                block.location.getZ().toFloat()
        )

        block.body.setWorldTransform(block.transform)
    }

    /**
     * Returns the remaining life of the block in seconds
     */
    public fun getLife(): Float {
        return block.life
    }

    /**
     * Sets the remaining life of the block in seconds
     */
    public fun setLife(life: Float) {
        block.life = life
    }

    /**
     * Applies the force to the block
     */
    public fun applyForce(vec : Vector) {
        block.body.setLinearVelocity(Vector3(
                vec.getX().toFloat(),
                vec.getY().toFloat(),
                vec.getZ().toFloat()
        ))
    }

    /**
     * Marks the block for removal
     */
    public fun kill() {
        block.stand.remove()
    }
}