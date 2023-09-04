from ursina import *
import pybullet as p

app = Ursina()

g = 9.81


# Start PyBullet
physicsClient = p.connect(p.DIRECT)  # Non-graphical mode
p.setGravity(0, -g, 0)

scale_factor = 0.01

earth_radius_game = 63710.00 * scale_factor
rocket_height_game = 2.00 * scale_factor
rocket_mass_game = 3050 * scale_factor


# Define the real-world distances for the atmospheric layers
troposphere_distance_real = 12000.00  # in  m
stratosphere_distance_real = 50000.00  # in  m
mesosphere_distance_real = 85000.00  # in  m
thermosphere_distance_real = 60000.00  # in m

# Convert the real-world distances (now in km) to the game's scale
troposphere_distance_game = troposphere_distance_real * scale_factor
stratosphere_distance_game = stratosphere_distance_real * scale_factor
mesosphere_distance_game = mesosphere_distance_real * scale_factor
thermosphere_distance_game = thermosphere_distance_real * scale_factor

# Calculate the radii of each layer based on the game's scale
troposphere_radius = earth_radius_game + troposphere_distance_game
stratosphere_radius = troposphere_radius + stratosphere_distance_game
mesosphere_radius = stratosphere_radius + mesosphere_distance_game
thermosphere_radius = mesosphere_radius + thermosphere_distance_game


troposphere_color = color.rgb(0, 0, 255, 100)  # Blue with alpha
stratosphere_color = color.rgb(150, 150, 255, 40)
mesosphere_color = color.rgb(80, 80, 200, 30)
thermosphere_color = color.rgb(40, 40, 150, 20)
space_color = color.rgb(0, 0, 0)

# Define the mass of Earth in your game's scale
earth_mass_game = 5.972 * 10**24 * scale_factor

moon_radius_game = 1737.1 * scale_factor
moon_mass_game = 7.35 * 10**22 * scale_factor**3

# Initial gimbal rotation angle (in degrees)
gimbal_rotation_angle = 0.0


rocket_diameter_game = 10.0 * scale_factor  # Assuming a diameter, adjust as needed
rocket_radius_game = rocket_diameter_game / 2

# Rocket thrust variables
thrust_kN = 845  # Thrust of Merlin 1D engine in kN
thrust_force = thrust_kN * 1000  # Convert kN to N
thrust_direction = Vec3(0, 1, 0)  # Direction of thrust (upwards in local coordinates)

# Flag to track if rocket is in space
in_space = False

Isp = 300  # specific impulse in seconds (for an RP-1/LOX engine)
density_RP1 = 810  # kg/m^3

# Define a volume for your rocket's fuel tank based on the game's scale and assuming a cylindrical shape
# Assuming 80% of the rocket's volume is the fuel tank (adjust this percentage as needed)
fuel_percentage = 0.8
V = math.pi * (rocket_radius_game**2) * rocket_height_game * fuel_percentage  # volume of a cylinder

# Calculate total propellant mass
m = V * density_RP1

# Estimate burn time in your game (you can adjust this as needed)
T = 10  # for instance, 10 seconds of burn time in the game

# Calculate mass flow rate
mass_flow_rate = m / T

# Calculate thrust
#thrust_force = Isp * g * mass_flow_rate


earth_radius = earth_radius_game
rocket_height = rocket_height_game

# Create the Earth (Sphere)
earth_position = [0, 0, 0]  # Earth is at the origin now

rocket_start_height = earth_radius + rocket_height / 2  # Above Earth's surface + half rocket's height + a small gap
rocket_position = [0, rocket_start_height, 0]

initial_vertical_position = rocket_start_height

# Define atmospheric drag constants
air_density = 1.225  # kg/m^3 (standard air density at sea level)
drag_coefficient = 0.47  # A typical drag coefficient for a streamlined object
frontal_area = math.pi * (rocket_radius_game**2)  # Using math.pi for the calculation


is_rocket_state = True
transition_speed = 2.0  # Adjust the transition speed as needed

moon_distance = earth_radius_game + 384400.0 * scale_factor

# Calculate the initial total fuel volume
initial_fuel_volume = math.pi * (rocket_radius_game**2) * rocket_height_game * fuel_percentage

# Calculate total propellant mass
initial_fuel_mass = initial_fuel_volume * density_RP1


def create_entities():
    global earth, moon, rocket, gimbal

    earth = Entity(model='sphere', color=color.rgb(139, 69, 19), scale=earth_radius_game)
    
    moon = Entity(model='sphere', color=color.gray, scale=moon_radius_game)
    moon_distance = earth_radius_game + 384400.0 * scale_factor
    moon.world_position = earth.world_position + Vec3(0, moon_distance, 0)
    
    rocket = Entity(parent=earth, model='soyuz-fg.glb', scale_y=rocket_height_game, scale_x=rocket_height_game, scale_z=rocket_height_game)
    rocket.scale *= 0.01
    rocket.world_position = earth.world_position + Vec3(0, earth_radius_game + rocket_height_game / 2, 0)
    
    gimbal = Entity(parent=rocket, model='cube', color=color.red, scale=(0.1, 0.1, 0.5))
    gimbal_position_offset = Vec3(0, -rocket_height_game / 2, 0)
    gimbal.world_position = rocket.world_position + gimbal_position_offset

create_entities()

# ===== Camera Setup =====

def setup_camera():
    camera.far = (earth_radius_game + moon_distance) * 2 * 10
    camera.near = 0.001
    camera.fov = 10
    camera.position = Vec3(rocket.world_position.x - 100, rocket.world_position.y, rocket.world_position.z + 100)
    camera.look_at(rocket)

setup_camera()


def initialize_ui():
    global button, oxygen_bar, oxygen_level

    button = Button(text='Combust', color=color.green, scale=(0.1, 0.05), position=(-0.6, 0.4))
    
    oxygen_bar = Entity(parent=camera.ui, model='quad', color=color.black, scale=(0.2, 0.05), position=(-0.6, -0.4))
    oxygen_level = Entity(parent=oxygen_bar, model='quad', color=color.white, scale=(1, 1), origin=(-0.5, 0), position=(-0.5, 0))

initialize_ui()

# ===== Physics Setup =====

def setup_physics():
    global earth_body, rocket_body
    
    earth_position = [0, 0, 0]
    earth_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=earth_radius_game)
    earth_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=earth_shape, basePosition=earth_position)
    
    rocket_position = [0, rocket_start_height, 0]
    rocket_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[rocket_height_game/10, rocket_height_game/2, rocket_height_game/10])
    rocket_body = p.createMultiBody(baseMass=rocket_mass_game, baseCollisionShapeIndex=rocket_shape, basePosition=rocket_position)

setup_physics()


original_camera_rotation = camera.rotation


# Update function
def update():
    global in_space, m, is_rocket_state, gimbal_rotation_angle

    # Adjust gimbal rotation based on input
    if held_keys['left arrow']:
        gimbal_rotation_angle += 1.0
    if held_keys['right arrow']:
        gimbal_rotation_angle -= 1.0


    # Limit gimbal rotation angle within a range
    gimbal_rotation_angle = clamp(gimbal_rotation_angle, -45.0, 45.0)
    
    # Calculate the thrust direction based on gimbal rotation
    gimbal_rotation_radians = math.radians(gimbal_rotation_angle)

    # Rotate the thrust force vector based on gimbal rotation
    thrust_direction = Vec3(0, math.cos(gimbal_rotation_radians), math.sin(gimbal_rotation_radians))


    thrust_force_vector = Vec3(
        thrust_direction.x * thrust_force,
        thrust_direction.y * thrust_force,
        thrust_direction.z * thrust_force
    )

    print(thrust_direction)
    print(thrust_force_vector)

    # Get the current altitude of the rocket
    altitude = rocket.world_position.y - earth_radius_game

    # Calculate the new gravity based on altitude using a simplified formula
    new_gravity = g * (earth_radius_game / (earth_radius_game + altitude))**2

    #print(new_gravity)

    # Set the new gravity
    p.setGravity(0, -new_gravity, 0)

    earth.rotation_y += time.dt * 10

    fuel_consumed_per_update = mass_flow_rate * time.dt
    fuel_fraction_left = m / (V * density_RP1)  # fraction of initial fuel left

    # Calculate the current total mass of the rocket (initial mass + remaining fuel)
    current_total_mass = (rocket_mass_game * 0.2) + (m / initial_fuel_mass) * (rocket_mass_game)

    # Update the bullet body mass
    p.changeDynamics(rocket_body, -1, mass=current_total_mass)


    if m > 0 and (held_keys['space'] or (button.hovered and held_keys['left mouse'])):
        # Only apply thrust and burn fuel if there's fuel left

        # Apply thrust force to the rocket's physics body
        p.applyExternalForce(rocket_body, -1, thrust_force_vector, rocket_position, p.LINK_FRAME)

        # Decrease the oxygen level to reflect fuel consumption
        oxygen_level.scale_x = fuel_fraction_left

        # Reduce remaining fuel
        m -= fuel_consumed_per_update

    if held_keys['1']:
        camera.fov = 10
        is_rocket_state = True

    if held_keys['2']:
        camera.fov = 60
        is_rocket_state = False

    if is_rocket_state:
        target_position = Vec3(rocket.world_position.x - 100, rocket.world_position.y, rocket.world_position.z + 100)
        target_rotation = original_camera_rotation
        #update_camera_position_and_rotation(target_position, target_rotation)
        camera.position = target_position
        camera.rotation = target_rotation
        camera.look_at(rocket)

    else:
        camera_distance = (earth_radius_game + moon_distance) * 1.5
        target_position = Vec3(camera_distance, 0, 0)
        target_rotation = original_camera_rotation
        #update_camera_position_and_rotation(target_position, target_rotation)
        camera.position = target_position
        camera.rotation = target_rotation
        camera.look_at((earth.world_position + moon.world_position) / 2)


    # Calculate drag force
    velocity = p.getBaseVelocity(rocket_body)[0]
    velocity_magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
    drag_force = 0.5 * air_density * velocity_magnitude**2 * drag_coefficient * frontal_area

    # Apply drag force in the opposite direction of velocity
    drag_force_direction = (-velocity[0] / velocity_magnitude, -velocity[1] / velocity_magnitude, -velocity[2] / velocity_magnitude) if velocity_magnitude > 0 else (0, 0, 0)
    drag_force_vec = (drag_force * drag_force_direction[0], drag_force * drag_force_direction[1], drag_force * drag_force_direction[2])
    p.applyExternalForce(rocket_body, -1, drag_force_vec, rocket_position, p.LINK_FRAME)


    pos, _ = p.getBasePositionAndOrientation(rocket_body)
    rocket.world_position = pos

    # Calculate gravitational force between Earth and Moon
    earth_to_moon_vector = moon.world_position - earth.world_position
    earth_to_moon_distance = earth_to_moon_vector.length()
    earth_to_moon_direction = earth_to_moon_vector.normalized()
#    gravitational_force = (g * (earth_mass_game * moon_mass_game) / (earth_to_moon_distance**2)) * earth_to_moon_direction

    # Apply gravitational forces to the rocket and the moon
#    p.applyExternalForce(rocket_body, -1, gravitational_force, rocket_position, p.LINK_FRAME)
#    p.applyExternalForce(moon_body, -1, -gravitational_force, moon_position, p.LINK_FRAME)


    #camera.position = Vec3(rocket.world_position.x - 100, rocket.world_position.y, rocket.world_position.z + 100)

    p.stepSimulation()

    # Check if rocket is above the thermosphere (last layer of atmosphere)
    if pos[1] > thermosphere_radius:
        in_space = True
    
    # Change the background color to black if in space
    
    # Calculate the altitude ratios for each atmospheric layer
    if altitude < troposphere_radius - earth_radius_game:
        ratio = altitude / (troposphere_radius - earth_radius_game)
        current_color = lerp(troposphere_color, stratosphere_color, ratio)
    elif altitude < stratosphere_radius - earth_radius_game:
        ratio = (altitude - (troposphere_radius - earth_radius_game)) / (stratosphere_radius - troposphere_radius)
        current_color = lerp(stratosphere_color, mesosphere_color, ratio)
    elif altitude < mesosphere_radius - earth_radius_game:
        ratio = (altitude - (stratosphere_radius - earth_radius_game)) / (mesosphere_radius - stratosphere_radius)
        current_color = lerp(mesosphere_color, thermosphere_color, ratio)
    else:
        ratio = (altitude - (mesosphere_radius - earth_radius_game)) / (thermosphere_radius - mesosphere_radius)
        current_color = thermosphere_color
    
    window.color = current_color

    distance_traversed = pos[1] - initial_vertical_position  # pos[1] gives the vertical position of the rocket
    print("Distance Traversed KM:", distance_traversed / scale_factor * 0.1)

app.run()
