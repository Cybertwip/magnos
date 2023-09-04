from ursina import *
import pybullet as p

app = Ursina()

# Start PyBullet
physicsClient = p.connect(p.DIRECT)  # Non-graphical mode
p.setGravity(0, -9.8, 0)

scale_factor = 0.1

# Determine a reasonable size_divisor
size_divisor = 1

earth_radius_game = 637.1 * scale_factor * size_divisor
rocket_height_game = 0.15 * scale_factor * size_divisor


print(rocket_height_game)

# Earth's layers
troposphere_radius = earth_radius_game + 15 * scale_factor
stratosphere_radius = troposphere_radius + 35 * scale_factor 
mesosphere_radius = stratosphere_radius + 35 * scale_factor
thermosphere_radius = mesosphere_radius + 1000 * scale_factor

# Create the Earth
earth = Entity(model='sphere', color=color.cyan, scale=earth_radius_game)
earth.collider = 'sphere'

# Create atmospheric layers
troposphere = Entity(model='sphere', color=color.rgb(255, 255, 255, 50), scale=troposphere_radius)
stratosphere = Entity(model='sphere', color=color.rgb(150, 150, 255, 40), scale=stratosphere_radius)
mesosphere = Entity(model='sphere', color=color.rgb(80, 80, 200, 30), scale=mesosphere_radius)
thermosphere = Entity(model='sphere', color=color.rgb(40, 40, 150, 20), scale=thermosphere_radius)

# Create the rocket
rocket = Entity(parent=earth, color=color.yellow, model='box', scale_y=rocket_height_game, scale_x=rocket_height_game/10, scale_z=rocket_height_game/10)
rocket.world_position = earth.world_position + Vec3(0, earth_radius_game + rocket_height_game / 2, 0)
rocket.collider = 'box'

# Camera setup
camera.far = earth_radius_game * size_divisor * 10  # Adjusted far plane value for better visibility
camera.near = 0.001
camera.fov = 60
camera.position = Vec3(rocket.world_position.x - 2, rocket.world_position.y, rocket.world_position.z)


earth_radius = earth_radius_game
rocket_height = rocket_height_game

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

thrust_force = 100  # Adjust this value as needed
thrust_direction = Vec3(0, 1, 0)  # Direction of thrust (upwards in local coordinates)

# Update function
def update():
    earth.rotation_y += time.dt * 10

    # Check if space key is held down
    if held_keys['space']:
        # Apply thrust force to the rocket's physics body
        p.applyExternalForce(rocket_body, -1, thrust_force * thrust_direction, rocket_position, p.LINK_FRAME)


    camera.position = Vec3(rocket.world_position.x - 2, rocket.world_position.y, rocket.world_position.z)

    pos, _ = p.getBasePositionAndOrientation(rocket_body)

    rocket.world_position = pos

    p.stepSimulation()

    camera.look_at(rocket)

app.run()
