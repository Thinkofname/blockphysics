package uk.thinkofdeath.minecraft.physics

import com.bulletphysics.collision.broadphase.DbvtBroadphase
import com.bulletphysics.collision.dispatch.CollisionDispatcher
import com.bulletphysics.collision.dispatch.CollisionFlags
import com.bulletphysics.collision.dispatch.CollisionObject
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration
import com.bulletphysics.collision.shapes.BoxShape
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape
import com.bulletphysics.dynamics.DiscreteDynamicsWorld
import com.bulletphysics.dynamics.RigidBody
import com.bulletphysics.dynamics.RigidBodyConstructionInfo
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver
import com.bulletphysics.linearmath.DefaultMotionState
import com.bulletphysics.linearmath.Transform
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.block.BlockState
import org.bukkit.entity.ArmorStand
import org.bukkit.entity.Player
import org.bukkit.event.Listener
import org.bukkit.event.entity.EntityExplodeEvent
import org.bukkit.event.player.PlayerChatEvent
import org.bukkit.event.player.PlayerJoinEvent
import org.bukkit.event.player.PlayerQuitEvent
import org.bukkit.inventory.ItemStack
import org.bukkit.event.EventHandler as event
import org.bukkit.plugin.java.JavaPlugin
import org.bukkit.util.EulerAngle
import java.util.*
import java.util.concurrent.TimeUnit
import javax.vecmath.Quat4f
import javax.vecmath.Vector3f

class PhysicsPlugin : JavaPlugin(), Listener {

    var collisionConfig = DefaultCollisionConfiguration()
    var dispatcher = CollisionDispatcher(collisionConfig)
    var broadphase = DbvtBroadphase()
    var solver = SequentialImpulseConstraintSolver()
    var dynamicsWorld = DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig)


    val boxCollision = BoxShape(Vector3f(.3f, .3f, .3f))
    val boxInertia = Vector3f(0f, 0f, 0f);
    val boxStaticCollision = BoxShape(Vector3f(1.0f, 1.0f, 1.0f))
    val playerCollision = BoxShape(Vector3f(.15f, 0.9f, .15f))
    val blocks = arrayListOf<PBlock>()

    val players = hashMapOf<Player, RigidBody>()
    var lastSim = System.nanoTime()

    public override fun onEnable() {
        dynamicsWorld.setGravity(Vector3f(0f, -10f, 0f))

        boxCollision.calculateLocalInertia(5f, boxInertia)

        getServer().getScheduler().runTaskTimer(this, { stepSimulation() }, 0, 1)
        getServer().getPluginManager().registerEvents(this, this)
    }

    public override fun onDisable() {
        blocks.forEach(PBlock::kill)
    }

    event fun explode(e : EntityExplodeEvent) {
        val loc = e.getLocation()
        blocks.forEach {
            val l = it.stand.getLocation()
            if (loc.distanceSquared(l) < e.getYield()*e.getYield()) {
                val vec = Vector3f(
                        (loc.getX() - l.getX()).toFloat(),
                        (loc.getY() - l.getY()).toFloat(),
                        (loc.getZ() - l.getZ()).toFloat()
                )
                vec.normalize()
                vec.scale(-e.getYield() * 30)
                it.body.setLinearVelocity(vec)
            }
        }
        val ite = e.blockList().listIterator()
        while (ite.hasNext()) {
            val it = ite.next()
            if (it.getType() == Material.TNT) {
                continue
            }
            val l = it.getLocation()
            val bl = PBlock(this, l, it.getState())
            blocks.add(bl)
            val vec = Vector3f(
                    (loc.getX() - l.getX()).toFloat(),
                    (loc.getY() - l.getY()).toFloat(),
                    (loc.getZ() - l.getZ()).toFloat()
            )
            vec.normalize()
            vec.scale(-e.getYield() * 30)
            bl.body.setLinearVelocity(vec)
            it.setType(Material.AIR)
            ite.remove()
        }
    }

    event fun playerJoin(e : PlayerJoinEvent) {
        val loc = e.getPlayer().getLocation()
        val transform = Transform()
        transform.setIdentity()
        transform.origin.set(
                loc.getX().toFloat(),
                loc.getY().toFloat() + 0.9f,
                loc.getZ().toFloat()
        )

        val motionState = DefaultMotionState(transform)
        val info = RigidBodyConstructionInfo(0f, motionState, playerCollision, Vector3f(0f, 0f, 0f))
        val body = RigidBody(info)
        dynamicsWorld.addRigidBody(body)
        players[e.getPlayer()] = body
    }

    event fun playerLeave(e : PlayerQuitEvent) {
        dynamicsWorld.removeRigidBody(players.remove(e.getPlayer()))
    }

    val pool = arrayListOf<RigidBody>()
    val visited = hashSetOf<Location>()

    fun stepSimulation() {
        var offset = 0

        blocks.forEach {
            if (!it.body.isActive()) {
                return@forEach
            }
            for (y in -2 .. 2) {
                for (z in -2 .. 2) {
                    for (x in -2 .. 2) {
                        val loc = it.location.clone().add(x.toDouble(),y.toDouble(),z.toDouble())
                        var b = loc.getBlock()
                        var bloc = b.getLocation()
                        if (b.getType() == Material.AIR || visited.contains(bloc)) {
                            continue
                        }
                        visited.add(bloc)

                        val transform = Transform()
                        transform.setIdentity()
                        transform.origin.set(
                                loc.getBlockX().toFloat() + .5f,
                                loc.getBlockY().toFloat() + .5f,
                                loc.getBlockZ().toFloat() + .5f
                        )

                        if (offset >= pool.size()) {
                            val motionState = DefaultMotionState(transform)
                            val info = RigidBodyConstructionInfo(0f, motionState, boxStaticCollision, Vector3f(0f, 0f, 0f))
                            val body = RigidBody(info)
                            body.setActivationState(0)
                            body.setCollisionFlags(body.getCollisionFlags() or CollisionFlags.STATIC_OBJECT)
                            dynamicsWorld.addRigidBody(body)
                            body.setActivationState(0)
                            pool.add(body)
                        }

                        val body = pool.get(offset)
                        body.setWorldTransform(transform)
                        body.setActivationState(CollisionObject.ACTIVE_TAG)

                        offset++
                    }
                }
            }
        }
        for (i in offset .. pool.size() - 1) {
            pool[i].setActivationState(CollisionObject.DISABLE_SIMULATION)
        }

        val t = Transform()
        t.setIdentity()
        players.forEach {
            val body = it.getValue()
            val p = it.getKey()
            val loc = p.getLocation()
            body.getCenterOfMassTransform(t)
            t.origin.set(
                    loc.getX().toFloat(),
                    loc.getY().toFloat()+ 0.9f,
                    loc.getZ().toFloat()
            )
            body.setCenterOfMassTransform(t)
            body.setActivationState(CollisionObject.ACTIVE_TAG)
            dynamicsWorld.debugDrawWorld()
        }

        val now = System.nanoTime()
        dynamicsWorld.stepSimulation((now - lastSim).toFloat() / TimeUnit.SECONDS.toNanos(1).toFloat())
        lastSim = now
        blocks.forEach(PBlock::tick)
        blocks.forEach({ if (!it.body.isActive()) {
            it.kill()
        } })
        blocks.removeIf({ !it.body.isActive() || it.stand.isDead() })
        visited.clear()
    }
}

class PBlock(val plugin : PhysicsPlugin, val location : Location, val block : BlockState) {

    val stand : ArmorStand
    val body : RigidBody
    val transform = Transform()
    val drops = block.getBlock().getDrops();

    init {
        location.setYaw(0f)
        location.setPitch(0f)
        stand = location.getWorld().spawn(location, javaClass<ArmorStand>())
        stand.setGravity(false)
        stand.setHelmet(block.getData().toItemStack())
        stand.setVisible(false)

        transform.setIdentity()
        transform.origin.set(
                location.getX().toFloat(),
                location.getY().toFloat() + 1.8f,
                location.getZ().toFloat()
        )

        val motionState = DefaultMotionState(transform)
        val info = RigidBodyConstructionInfo(5f, motionState, plugin.boxCollision, plugin.boxInertia)
        body = RigidBody(info)
        plugin.dynamicsWorld.addRigidBody(body)
    }

    fun tick() {
        body.getMotionState().getWorldTransform(transform)

        var rot = Quat4f()
        transform.getRotation(rot)
        rot.y += Math.PI.toFloat()
        stand.setHeadPose(EulerAngle(
                rot.x.toDouble(),
                rot.y.toDouble(),
                rot.z.toDouble()
        ))


        location.setX(transform.origin.x.toDouble())
        location.setY(transform.origin.y.toDouble() - 1.8f + 0.1f)
        location.setZ(transform.origin.z.toDouble())

        location.add(
                -Math.sin(rot.z.toDouble()) * 0.15,
                -Math.cos(rot.x.toDouble()) * 0.15 -Math.cos(rot.z.toDouble()) * 0.15,
                -Math.sin(rot.x.toDouble()) * 0.15
        )

        if (location.getY() < - 10 || location.getY() > 300 || stand.isDead()) {
            kill()
        }
        stand.teleport(location)
    }

    fun kill() {
        stand.remove()
        plugin.dynamicsWorld.removeRigidBody(body)
        stand.remove()
        val loc = stand.getLocation()
        drops.forEach {
            loc.getWorld().dropItemNaturally(loc, it)
        }
    }
}