// Copyright 2015 Matthew Collins
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package uk.thinkofdeath.minecraft.physics

import com.badlogic.gdx.math.Matrix4
import com.badlogic.gdx.math.Quaternion
import com.badlogic.gdx.math.Vector3
import com.badlogic.gdx.physics.bullet.Bullet
import com.badlogic.gdx.physics.bullet.collision.*
import com.badlogic.gdx.physics.bullet.dynamics.btDiscreteDynamicsWorld
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody
import com.badlogic.gdx.physics.bullet.dynamics.btSequentialImpulseConstraintSolver
import com.badlogic.gdx.physics.bullet.linearmath.btDefaultMotionState
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.block.BlockState
import org.bukkit.entity.ArmorStand
import org.bukkit.entity.Player
import org.bukkit.event.Listener
import org.bukkit.event.entity.EntityExplodeEvent
import org.bukkit.event.player.PlayerJoinEvent
import org.bukkit.event.player.PlayerQuitEvent
import org.bukkit.plugin.java.JavaPlugin
import org.bukkit.util.EulerAngle
import java.util.concurrent.TimeUnit
import org.bukkit.event.EventHandler as event

val armorBlockSize = 0.625f
val armorBlockSizeH = armorBlockSize / 2
// .4375f

class PhysicsPlugin : JavaPlugin(), Listener {
    init {
        Bullet.init()
    }

    var collisionConfig = btDefaultCollisionConfiguration()
    var dispatcher = btCollisionDispatcher(collisionConfig)
    var broadphase = btDbvtBroadphase()
    var solver = btSequentialImpulseConstraintSolver()
    var dynamicsWorld = btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig)


    val boxCollision = btBoxShape(Vector3(armorBlockSizeH, armorBlockSizeH, armorBlockSizeH))
    val boxInertia = Vector3(0f, 0f, 0f);
    val boxStaticCollision = btBoxShape(Vector3(1.0f, 1.0f, 1.0f))
    val playerCollision = btBoxShape(Vector3(.15f, 0.9f, .15f))
    val blocks = arrayListOf<PBlock>()

    val players = hashMapOf<Player, btRigidBody>()
    var lastSim = System.nanoTime()

    var trackExplosions = true

    public override fun onEnable() {
        dynamicsWorld.setGravity(Vector3(0f, -10f, 0f))

        boxCollision.calculateLocalInertia(30f, boxInertia)

        getServer().getScheduler().runTaskTimer(this, { stepSimulation() }, 0, 1)
        getServer().getPluginManager().registerEvents(this, this)
        saveDefaultConfig()
        val config = getConfig();
        trackExplosions = config.getBoolean("explosions")
    }

    public override fun onDisable() {
        blocks.forEach(PBlock::kill)
    }

    event fun explode(e: EntityExplodeEvent) {
        if (!trackExplosions) return
        val loc = e.getLocation()
        blocks.forEach {
            val l = it.stand.getLocation()
            if (loc.distanceSquared(l) < e.getYield() * e.getYield() * e.getYield()) {
                val vec = Vector3(
                        (loc.getX() - l.getX()).toFloat(),
                        (loc.getY() - l.getY()).toFloat(),
                        (loc.getZ() - l.getZ()).toFloat()
                )
                vec.nor()
                vec.scl(-e.getYield() * 50)
                it.body.setLinearVelocity(vec)
            }
        }
        val ite = e.blockList().listIterator()
        while (ite.hasNext()) {
            val it = ite.next()
            if (it.getType() == Material.TNT) {
                continue
            }
            val l = it.getLocation().add(0.5,-0.5,0.5)
            val bl = PBlock(this, l, it.getState())
            blocks.add(bl)
            val vec = Vector3(
                    (loc.getX() - l.getX()).toFloat(),
                    (loc.getY() - l.getY()).toFloat(),
                    (loc.getZ() - l.getZ()).toFloat()
            )
            vec.nor()
            vec.scl(-e.getYield() * 50)
            bl.body.setLinearVelocity(vec)
            it.setType(Material.AIR)
            ite.remove()
        }
    }

    event fun playerJoin(e: PlayerJoinEvent) {
        val loc = e.getPlayer().getLocation()
        val transform = Matrix4()
        transform.idt()
        transform.setTranslation(
                loc.getX().toFloat(),
                loc.getY().toFloat() + 0.9f,
                loc.getZ().toFloat()
        )

        val motionState = btDefaultMotionState(transform)
        val info = btRigidBody.btRigidBodyConstructionInfo(0f, motionState, playerCollision, Vector3(0f, 0f, 0f))
        val body = btRigidBody(info)
        dynamicsWorld.addRigidBody(body)
        players[e.getPlayer()] = body
    }

    event fun playerLeave(e: PlayerQuitEvent) {
        dynamicsWorld.removeRigidBody(players.remove(e.getPlayer()))
    }

    val pool = arrayListOf<btRigidBody>()
    val visited = hashSetOf<Location>()

    fun stepSimulation() {
        var offset = 0

        blocks.forEach {
            if (!it.body.isActive()) {
                return@forEach
            }
            for (y in -2..2) {
                for (z in -2..2) {
                    for (x in -2..2) {
                        val loc = it.location.clone().add(x.toDouble(), y.toDouble(), z.toDouble())
                        var b = loc.getBlock()
                        var bloc = b.getLocation(loc)
                        if (b.getType() == Material.AIR || !b.getType().isOccluding() || visited.contains(bloc)) {
                            continue
                        }
                        visited.add(bloc)

                        val transform = Matrix4()
                        transform.idt()
                        transform.setTranslation(
                                loc.getBlockX().toFloat() + .5f,
                                loc.getBlockY().toFloat() + .5f,
                                loc.getBlockZ().toFloat() + .5f
                        )

                        if (offset >= pool.size()) {
                            val info = btRigidBody.btRigidBodyConstructionInfo(0f, null, boxStaticCollision, Vector3(0f, 0f, 0f))
                            val body = btRigidBody(info)
                            body.setActivationState(0)
                            body.setCollisionFlags(body.getCollisionFlags() or btCollisionObject.CollisionFlags.CF_STATIC_OBJECT)
                            pool.add(body)
                        }

                        val body = pool.get(offset)
                        body.setWorldTransform(transform)
                        dynamicsWorld.removeRigidBody(body)
                        dynamicsWorld.addRigidBody(body)
                        body.setActivationState(0)

                        offset++
                    }
                }
            }
        }
        for (i in offset..pool.size() - 1) {
            pool[i].setActivationState(CollisionConstants.DISABLE_SIMULATION)
        }

        players.forEach {
            val body = it.getValue()
            val p = it.getKey()
            val loc = p.getLocation()
            val t = body.getCenterOfMassTransform()
            t.setTranslation(
                    loc.getX().toFloat(),
                    loc.getY().toFloat() + 0.9f,
                    loc.getZ().toFloat()
            )
            body.setCenterOfMassTransform(t)
            body.setActivationState(CollisionConstants.ACTIVE_TAG)
        }

        val now = System.nanoTime()
        val delta = (now - lastSim).toFloat() / TimeUnit.SECONDS.toNanos(1).toFloat()
        dynamicsWorld.stepSimulation(delta, 100)
        lastSim = now
        val ite = blocks.iterator()
        while (ite.hasNext()) {
            val it = ite.next()
            it.tick(delta)
            if (!it.body.isActive() || it.stand.isDead()) {
                it.kill()
            }
            if (it.body.isDisposed()) {
                ite.remove()
            }
        }
        visited.clear()
    }
}

class PBlock(val plugin: PhysicsPlugin, val location: Location, val block: BlockState) {

    val stand: ArmorStand
    val body: btRigidBody
    val transform = Matrix4()
    val drops = block.getBlock().getDrops();
    var life = 60f

    init {
        location.setYaw(0f)
        location.setPitch(0f)
        stand = location.getWorld().spawn(location, javaClass<ArmorStand>())
        stand.setGravity(false)
        stand.setHelmet(block.getData().toItemStack())
        stand.setVisible(false)

        transform.idt()
        transform.setTranslation(
                location.getX().toFloat(),
                location.getY().toFloat() + 1.8f,
                location.getZ().toFloat()
        )

        val motionState = btDefaultMotionState(transform)
        val info = btRigidBody.btRigidBodyConstructionInfo(30f, motionState, plugin.boxCollision, plugin.boxInertia)
        info.setAdditionalDamping(true)
        body = btRigidBody(info)
        info.dispose()
        plugin.dynamicsWorld.addRigidBody(body)
    }

    fun tick(delta: Float) {
        body.getMotionState().getWorldTransform(transform)

        var rot = Quaternion()
        transform.getRotation(rot)
        val eul = quatToEul(rot)
        stand.setHeadPose(eul)


        val origin = transform.getTranslation(Vector3())
        location.setX(origin.x.toDouble())
        location.setY(origin.y.toDouble() - 1.8f + 0.05f)
        location.setZ(origin.z.toDouble())

        location.add(
                -Math.sin(eul.getZ().toDouble()) * (armorBlockSizeH/2),
                -Math.cos(eul.getX().toDouble()) * (armorBlockSizeH/2) - Math.cos(eul.getZ().toDouble()) * (armorBlockSizeH/2),
                -Math.sin(eul.getX().toDouble()) * (armorBlockSizeH/2)
        )

        life -= delta
        if (location.getY() < -10 || location.getY() > 300 || stand.isDead() || life < 0) {
            stand.remove()
        }
        stand.teleport(location)
    }

    fun kill() {
        if (body.isDisposed()) {
            return
        }
        stand.remove()
        plugin.dynamicsWorld.removeRigidBody(body)
        val loc = stand.getLocation()
        drops.forEach {
            loc.getWorld().dropItemNaturally(loc, it)
        }
        body.dispose()
    }
}

fun quatToEul(q: Quaternion): EulerAngle {
    val sqw = q.w * q.w
    val sqx = q.x * q.x
    val sqy = q.y * q.y
    val sqz = q.z * q.z
    val unit = sqx + sqy + sqz + sqw
    val test = q.x * q.y + q.z * q.w
    if (test > 0.499 * unit) {
        return EulerAngle(
                Math.PI / 2,
                2 * Math.atan2(q.x.toDouble(), q.w.toDouble()),
                0.0
        )
    }
    if (test < -0.499 * unit) {
        return EulerAngle(
                -Math.PI / 2,
                -2 * Math.atan2(q.x.toDouble(), q.w.toDouble()),
                0.0
        )
    }

    return EulerAngle(
            Math.atan2(
                    (2 * q.y * q.w - 2 * q.x * q.z).toDouble(),
                    (sqx - sqy - sqz + sqw).toDouble()
            ),
            -Math.atan2(
                    (2 * q.x * q.w - 2 * q.y * q.z).toDouble(),
                    (-sqx + sqy - sqz + sqw).toDouble()
            ),
            -Math.asin((2 * test / unit).toDouble())
    )
}