import pybullet as p
import time
from ursina import *
from panda3d.core import LMatrix4f, LVecBase3f

import math

def quaternion_to_euler(quaternion):
    # Extract the values from the quaternion
    w, x, y, z = quaternion

    # Compute yaw (yaw), pitch (pitch), and roll (roll) from the quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]  # Convert to degrees


class FlyingBox(Entity):
    def __init__(self):
        super().__init__(
            model='cube',
            color=color.azure,
            scale=(1,1,1)
        )

        self.force_magnitude = 100    
        
        # Create collision shape and then multibody
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        self.physics_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=collision_shape)

        # Introduce direction for gimbaled thrust
        self.thrust_direction = Vec3(0, 1, 0)  # By default, thrust upwards
    
    def adjust_thrust_direction(self, yaw, pitch):
        # Modify thrust direction based on yaw and pitch adjustments.
        
        # In Ursina, we rotate around the Z-axis for yaw and X-axis for pitch

        yaw_matrix = LMatrix4f.rotateMat(yaw, Vec3(0, 0, 1))  # Using Vec3 to represent the forward direction

        pitch_matrix = LMatrix4f.rotateMat(pitch, Vec3(1, 0, 0))
        
        rotation_matrix = yaw_matrix * pitch_matrix
        self.thrust_direction = rotation_matrix.xformVec(self.thrust_direction)
        self.thrust_direction.normalize()
        


    def update(self):
        force = Vec3(0, 0, 0)

        # Adjust thrust direction
        yaw = held_keys['q'] * -0.1 - held_keys['e'] * 0.1  # Q and E for yaw
        pitch = held_keys['r'] * 0.1 - held_keys['f'] * -0.1  # R and F for pitch
        self.adjust_thrust_direction(yaw, pitch)

        if held_keys['space']:
            # Apply thrust in the direction of gimbaled thrust
            force += self.thrust_direction * self.force_magnitude

        p.applyExternalForce(self.physics_id, -1, [force.x, force.y, force.z], [0, 0, 0], p.WORLD_FRAME)

# Initialize Ursina app
app = Ursina()

# Initialize PyBullet
p.connect(p.DIRECT)
p.setGravity(0, 0, 0)  # No gravity for the purpose of this example

# Create the flying box
box = FlyingBox()

def update():
    p.stepSimulation()
    pos, orn = p.getBasePositionAndOrientation(box.physics_id)
    box.position = Vec3(pos)
    _, orn = p.getBasePositionAndOrientation(box.physics_id)
    box.rotation = quaternion_to_euler(orn)

    print(box.rotation)

    camera.position = box.position + Vec3(-10, 0, 0)
    camera.look_at(box)

app.run()






