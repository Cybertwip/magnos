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



class Earth:
    def __init__(self):
        self.entity = Entity(model='sphere', color=color.rgb(139, 69, 19), scale=earth_radius_game)
        self.rotation_speed = 10

    def update(self):
        self.entity.rotation_y += time.dt * self.rotation_speed


class Rocket:
    def __init__(self, body):
        self.entity = Entity(parent=earth.entity, model='soyuz-fg.glb', scale_y=rocket_height_game, scale_x=rocket_height_game, scale_z=rocket_height_game)
        self.entity.scale *= 0.01
        self.entity.world_position = earth.entity.world_position + Vec3(0, earth_radius_game + rocket_height_game / 2, 0)
        self.rocket_body = body
        self.gimbal_rotation_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0

        self.gimbal = Entity(parent=self.entity, model='cube', color=color.red, scale=(0.1, 0.1, 0.5))
        self.gimbal_position_offset = Vec3(0, -rocket_height_game / 2, 0)
        self.gimbal.world_position = self.entity.world_position + self.gimbal_position_offset


    def update(self):
        self._adjust_gimbal_rotation()
        self._calculate_thrust_direction()
        self._apply_gravity()
        self._consume_fuel_and_apply_thrust()
        self._apply_drag_force()
        self._update_position()

    def _adjust_gimbal_rotation(self):
        global held_keys

        if held_keys['left arrow']:
            self.yaw_angle += 1.0
        if held_keys['right arrow']:
            self.yaw_angle -= 1.0
        if held_keys['up arrow']:
            self.pitch_angle += 1.0
        if held_keys['down arrow']:
            self.pitch_angle -= 1.0

        # Limit gimbal rotation angles within a range
        self.pitch_angle = clamp(self.pitch_angle, -45.0, 45.0)
        self.yaw_angle = clamp(self.yaw_angle, -45.0, 45.0)

    def _calculate_thrust_direction(self):
        pitch_radians = math.radians(self.pitch_angle)
        yaw_radians = math.radians(self.yaw_angle)
        
        # Calculate the direction of thrust using both pitch and yaw adjustments.
        # Note: The exact trigonometric transformations depend on how your 3D coordinate system is set up.
        # Here's a common transformation for yaw and pitch:
        x_dir = math.sin(pitch_radians) * math.cos(yaw_radians)
        y_dir = math.cos(pitch_radians)
        z_dir = math.sin(pitch_radians) * math.sin(yaw_radians)
        
        self.thrust_direction = Vec3(x_dir, y_dir, z_dir)

    def _apply_gravity(self):
        # Get the current altitude of the rocket
        altitude = self.entity.world_position.y - earth_radius_game

        # Calculate the new gravity based on altitude using a simplified formula
        new_gravity = g * (earth_radius_game / (earth_radius_game + altitude))**2

        # Set the new gravity
        p.setGravity(0, -new_gravity, 0)

    def _consume_fuel_and_apply_thrust(self):
        global m, thrust_force, held_keys, button, oxygen_level, initial_fuel_mass

        fuel_consumed_per_update = mass_flow_rate * time.dt
        fuel_fraction_left = m / (V * density_RP1)  # fraction of initial fuel left

        # Calculate the current total mass of the rocket (initial mass + remaining fuel)
        current_total_mass = (rocket_mass_game * 0.2) + (m / initial_fuel_mass) * (rocket_mass_game)

        # Update the bullet body mass
        p.changeDynamics(self.rocket_body, -1, mass=current_total_mass)

        # thrust_force_vector = Vec3(
        #     self.thrust_direction.x * thrust_force,
        #     self.thrust_direction.y * thrust_force,
        #     self.thrust_direction.z * thrust_force
        # )
        thrust_force_vector = Vec3(
                                    0,
                                    thrust_force,
                                    0
        )
        
        if m > 0 and (held_keys['space'] or (button.hovered and held_keys['left mouse'])):
            # Apply thrust force to the rocket's physics body
            p.applyExternalForce(self.rocket_body, -1, thrust_force_vector, self.gimbal.world_position, p.LINK_FRAME)

            # Decrease the oxygen level to reflect fuel consumption
            oxygen_level.scale_x = fuel_fraction_left

            # Reduce remaining fuel
            m -= fuel_consumed_per_update

    def _apply_drag_force(self):
        # Calculate drag force
        velocity = p.getBaseVelocity(self.rocket_body)[0]
        velocity_magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        drag_force = 0.5 * air_density * velocity_magnitude**2 * drag_coefficient * frontal_area

        # Apply drag force in the opposite direction of velocity
        drag_force_direction = (-velocity[0] / velocity_magnitude, -velocity[1] / velocity_magnitude, -velocity[2] / velocity_magnitude) if velocity_magnitude > 0 else (0, 0, 0)
        drag_force_vec = (drag_force * drag_force_direction[0], drag_force * drag_force_direction[1], drag_force * drag_force_direction[2])
        p.applyExternalForce(self.rocket_body, -1, drag_force_vec, rocket_position, p.LINK_FRAME)

    def _update_position(self):
        pos, _ = p.getBasePositionAndOrientation(self.rocket_body)
        self.entity.world_position = pos
        self.gimbal.world_position = self.entity.world_position + self.gimbal_position_offset


class Moon:
    def __init__(self):
        self.entity = Entity(model='sphere', color=color.gray, scale=moon_radius_game)
        self.entity.world_position = earth.entity.world_position + Vec3(0, moon_distance, 0)

    def update(self):
        pass

class GameCamera:
    def __init__(self):
        self.entity = camera
        self.original_rotation = self.entity.rotation
        self.is_rocket_state = True  # default to rocket state

    def update(self):
        if held_keys['1']:
            self.entity.fov = 10
            self.is_rocket_state = True

        if held_keys['2']:
            self.entity.fov = 60
            self.is_rocket_state = False

        self._update_position_and_rotation()

    def _update_position_and_rotation(self):
        if self.is_rocket_state:
            target_position = Vec3(rocket.entity.world_position.x - 50, rocket.entity.world_position.y, rocket.entity.world_position.z + 50)
            target_rotation = self.original_rotation
            self._set_camera(target_position, target_rotation)
            self.entity.look_at(rocket.entity)
        else:
            camera_distance = (earth_radius_game + moon_distance) * 1.5
            target_position = Vec3(camera_distance, 0, 0)
            target_rotation = self.original_rotation
            self._set_camera(target_position, target_rotation)
            self.entity.look_at((earth.entity.world_position + moon.entity.world_position) / 2)

    def _set_camera(self, position, rotation):
        self.entity.position = position
        self.entity.rotation = rotation


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

global earth, rocket, moon
earth = Earth()
moon = Moon()

# After creating other instances
rocket = Rocket(rocket_body)

game_camera = GameCamera()

# Update function
def update():
    rocket.update()
    earth.update()
    moon.update()
    game_camera.update()
    p.stepSimulation()
    
    altitude = rocket.entity.world_position.y - earth_radius_game

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


app.run()
