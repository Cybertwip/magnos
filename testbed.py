import pybullet as p
import time

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # GUI mode for visualization

scale_factor = 0.01  # This will scale the Earth's radius down to 6.371 (from 6371)

# Set up the environment
p.setGravity(0, -9.8, 0)
p.setRealTimeSimulation(0)  # We will step the simulation ourselves

# Dimensions
earth_radius = 6371000 * scale_factor
rocket_height = 100 * scale_factor

# Create the Earth (Sphere)
earth_position = [0, 0, 0]  # Earth is at the origin now
earth_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=earth_radius)
earth_body = p.createMultiBody(baseMass=0,  # Static
                               baseCollisionShapeIndex=earth_shape,
                               basePosition=earth_position)

# Create the Rocket (Box)
rocket_start_height = earth_radius + rocket_height / 2  # Above Earth's surface + half rocket's height + a small gap
rocket_position = [0, rocket_start_height, 0]
rocket_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[rocket_height/10, rocket_height/2, rocket_height/10])
rocket_body = p.createMultiBody(baseMass=1,
                                baseCollisionShapeIndex=rocket_shape,
                                basePosition=rocket_position)

p.resetDebugVisualizerCamera(cameraDistance=3 * earth_radius, cameraYaw=0, cameraPitch=90, cameraTargetPosition=[0, 0, 0])

# Run simulation
try:
    while True:
        pos, _ = p.getBasePositionAndOrientation(rocket_body)

        print(pos)

        p.stepSimulation()
        time.sleep(1./240.)  # Running at 240hz real time
except KeyboardInterrupt:
    p.disconnect()
