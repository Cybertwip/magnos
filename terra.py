from ursina import *

import pybullet as p

app = Ursina()

g = 9.81


# Start PyBullet

physics_client = p.connect(p.DIRECT)

p.setGravity(0, -g, 0)

# Ensure connection was successful.
if physics_client == -1:
    print("Failed to connect to pybullet.")
    exit()


scale_factor = 0.01

earth_radius_game = 63710.00 * scale_factor
rocket_height_game = 2.00 * scale_factor
rocket_mass_game = 305000 * scale_factor


# Define the real-world distances for the atmospheric layers
troposphere_distance_real = 1200.00  # in  m
stratosphere_distance_real = 5000.00  # in  m
mesosphere_distance_real = 8500.00  # in  m
thermosphere_distance_real = 6000.00  # in m

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
exosphere_color = color.rgb(0, 0, 0)

# Define the mass of Earth in your game's scale
earth_mass_game = 5.972 * 10**24 * scale_factor

moon_radius_game = 17371 * scale_factor
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

moon_distance = earth_radius_game + 38440000.0 * scale_factor

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
        self.entity = Entity(parent=earth.entity, scale_y=rocket_height_game, scale_x=rocket_height_game, scale_z=rocket_height_game)

        self.rocket = Entity(parent=self.entity, model='soyuz-fg.glb', scale_y=1, scale_x=1, scale_z=1)

        self.rocket.scale *= 1 * scale_factor

        self.entity.world_position = earth.entity.world_position + Vec3(0, earth_radius_game + rocket_height_game / 2, 0)
        self.rocket_body = body
        self.gimbal_rotation_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0

        self.gimbal = Entity(parent=self.entity, model='cube', color=color.red, scale=(0.01, 0.01, 0.05))
        self.gimbal_position_offset = Vec3(0, -rocket_height_game / 2, 0)
        self.gimbal.world_position = self.entity.world_position + self.gimbal_position_offset

        # Defining stage masses
        self.first_stage_mass = rocket_mass_game * 0.4  # Example mass distribution
        self.second_stage_mass = rocket_mass_game * 0.35
        self.third_stage_mass = rocket_mass_game * 0.25
        
        # Starting with all stages attached
        self.current_mass = rocket_mass_game
        
        scale = 10 * scale_factor

        # Representing stages with simple shapes (e.g., cylinders)
        # Set the parent at the time of creation.
        self.first_stage = Entity(parent=self.entity,
                                model='cube', 
                                scale=(scale, scale, scale), 
                                color=color.red)

        # Set the parent at the time of creation.
        self.second_stage = Entity(parent=self.entity,
                                model='cube', 
                                scale=(scale, scale, scale), 
                                color=color.green)

        # Position the second_stage just above the top of self.entity. 
        # If the height of self.entity is based on its y-scale, then:
        self.second_stage.y = (self.entity.scale.y / 4 / self.rocket.scale.y) * 0.4


        # Set the parent at the time of creation.
        # self.third_stage = Entity(parent=self.entity,
        #                         model='cube', 
        #                         scale=(scale, scale, scale), 
        #                         color=color.cyan)

        # Position the second_stage just above the top of self.entity. 
        # If the height of self.entity is based on its y-scale, then:
        #self.third_stage.y = (self.entity.scale.y / 4 / self.rocket.scale.y) * 0.75


        self.rocket_body = body
        self.gimbal_rotation_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0


        self.initial_position = self.entity.world_position
        self.previous_position = self.entity.world_position

        self.traveled_distance = 0
        self.current_fuel_mass = initial_fuel_mass

        self.current_stage = 1  # Starting stage

        self.frame_counter = 0  # Counting frames since the last UI update
        self.previous_velocity = (0, 0, 0)  # Assuming it starts at rest

    def jettison_first_stage(self):
        if self.first_stage:
            self.first_stage.disable()  # Assuming a method to make the entity inactive or remove it
            self.current_mass -= self.first_stage_mass
            self._update_mass()
            self.first_stage = None  # Remove reference so we can't jettison it again
            self.current_stage = 2  # Move to the second stage

    def jettison_second_stage(self):
        if self.second_stage:
            self.second_stage.disable()  # Remove or disable the visual representation
            self.current_mass -= self.second_stage_mass
            self._update_mass()
            self.second_stage = None  # Remove reference
            self.current_stage = 3  # Move to the third stage


    def update(self):
        self._adjust_gimbal_rotation()
        self._calculate_thrust_direction()
        self._apply_gravity()
        self._consume_fuel_and_apply_thrust()
        self._apply_drag_force()
        self._update_position()

        self.frame_counter += 1

        # Update the UI elements every 10 frames (or whatever number you prefer)
        if self.frame_counter % 6 == 0:
            altitude = self.entity.world_position.y - earth_radius_game 
            altitude = altitude / 1000 / scale_factor
            # Determine the current atmospheric layer
            if altitude <= 12:
                atmospheric_layer = "Troposphere"
            elif altitude <= 50:
                atmospheric_layer = "Stratosphere"
            elif altitude <= 85:
                atmospheric_layer = "Mesosphere"
            elif altitude <= 600:
                atmospheric_layer = "Thermosphere"
            else:
                atmospheric_layer = "Exosphere"

            velocity = p.getBaseVelocity(self.rocket_body)[0]  # Assuming this returns a velocity vector

            dt = time.dt
            speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2) # Calculate the magnitude of the velocity vector

            distance_traveled = speed * dt

            self.traveled_distance += distance_traveled * scale_factor * 10

            self.previous_position = self.entity.world_position  # Store the current position for the next frame

            speed_label.text = f"SPEED: {speed:.2f} m/s" 

            # Compute acceleration magnitude
            acceleration = math.sqrt((velocity[0] - self.previous_velocity[0])**2 + 
                                    (velocity[1] - self.previous_velocity[1])**2 + 
                                    (velocity[2] - self.previous_velocity[2])**2) / dt


            acceleration_label.text = f"ACCEL: {acceleration:.2f} m/s^2"

            
            self.previous_velocity = velocity  # Store current velocity for next frame


            atmospheric_layer_label.text = f"LAYER: {atmospheric_layer}"


            altitude = self.entity.world_position.y - earth_radius_game 
            altitude = altitude / 1000 / scale_factor

            fuel_percentage = (self.current_fuel_mass / initial_fuel_mass) * 100  # Calculate the percentage of fuel remaining

            altitude_label.text = f"ALTITUDE: {altitude:.2f} KM"  # Rounded to two decimal places for display
            distance_label.text = f"DISTANCE: {self.traveled_distance:.2f} KM"  # Rounded to two decimal places for display

            fuel_label.text = f"FUEL: {fuel_percentage:.2f}%"  # Display the fuel percentage
        
            # To calculate the speed, we need the change in position divided by time.
            current_stage_label.text = f"STAGE: {self.current_stage}"

            info = p.getDynamicsInfo(self.rocket_body, -1)
            current_mass = info[0] / scale_factor # Mass is the first item in the tuple returned by getDynamicsInfo
            mass_label.text = f"MASS: {current_mass:.2f} kg"

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

        # Jettison stages with number keys
        if held_keys['1']:
            self.jettison_first_stage()
        if held_keys['2']:
            self.jettison_second_stage()


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


    def _update_mass(self):
        # Calculate the current total mass of the rocket (initial mass + remaining fuel)
        current_total_mass = (self.current_mass * 0.2) + (m / initial_fuel_mass) * (rocket_mass_game)

        # Update the bullet body mass
        p.changeDynamics(self.rocket_body, -1, mass=current_total_mass, linearDamping=0.0, angularDamping=0.0, maxJointVelocity = 726)

    def _consume_fuel_and_apply_thrust(self):
        global m, thrust_force, held_keys, oxygen_level, initial_fuel_mass

        fuel_consumed_per_update = min(self.current_fuel_mass, mass_flow_rate * time.dt)
        fuel_fraction_left = m / (V * density_RP1)  # fraction of initial fuel left


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
        global start_time
        
        if m > 0 and held_keys['space']:
            # Apply thrust force to the rocket's physics body
            p.applyExternalForce(self.rocket_body, -1, thrust_force_vector, self.gimbal.world_position, p.LINK_FRAME)

            # Decrease the oxygen level to reflect fuel consumption
            oxygen_level.scale_x = fuel_fraction_left

            # Reduce remaining fuel
            m -= fuel_consumed_per_update
            self.current_fuel_mass -= fuel_consumed_per_update  # Decrease current fuel mass

            if start_time is None:
                start_time = time.time()  # Record the start time when space is first pressed

        if start_time:
            elapsed_time = time.time() - start_time
            minutes, seconds = divmod(int(elapsed_time), 60)
            timer_label.text = f"TIMER: {minutes:02}:{seconds:02}"


        self._update_mass()


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
        if held_keys['o']:
            self.entity.fov = 10
            self.is_rocket_state = True

        if held_keys['p']:
            self.entity.fov = 60
            self.is_rocket_state = False

        self._update_position_and_rotation()

    def _update_position_and_rotation(self):
        if self.is_rocket_state:
            target_position = Vec3(rocket.entity.world_position.x - 50, rocket.entity.world_position.y, rocket.entity.world_position.z + 50)
            #target_rotation = self.original_rotation
            self._set_camera(target_position)
            self.entity.look_at(rocket.entity)
        else:
            # Calculate the midpoint between Earth and Moon
            midpoint = (earth.entity.world_position + moon.entity.world_position) / 2

            # Set the camera's target position laterally so that it encompasses both Earth and Moon
            target_position = Vec3(midpoint.y * 2, midpoint.x, midpoint.z)

            self._set_camera(target_position)
            self.entity.look_at(midpoint)

    def _set_camera(self, position):
        self.entity.position = position
        #self.entity.rotation = rotation


def initialize_ui():
    global oxygen_bar, oxygen_level, altitude_label, distance_label, fuel_label, speed_label, current_stage_label, atmospheric_layer_label, mass_label, timer_label, start_time, acceleration_label

    oxygen_bar = Entity(parent=camera.ui, model='quad', color=color.black, scale=(0.2, 0.05), position=(-0.6, -0.4))
    oxygen_level = Entity(parent=oxygen_bar, model='quad', color=color.white, scale=(1, 1), origin=(-0.5, 0), position=(-0.5, 0))
    altitude_label = Text(text="ALTITUDE: 0", position=(-0.75, 0.45))
    distance_label = Text(text="DISTANCE: 0", position=(-0.75, 0.4))
    fuel_label = Text(text="FUEL: 100%", position=(-0.75, 0.35))
    speed_label = Text(text="SPEED: 0 m/s", position=(-0.75, 0.3))
    acceleration_label = Text(text="ACCEL: 0 m/s", position=(-0.75, 0.25))
    current_stage_label = Text(text="STAGE: 1", position=(-0.75, 0.2))  # Assuming you start with Stage 1

    atmospheric_layer_label = Text(text="LAYER: Troposphere", position=(-0.75, 0.15))
    mass_label = Text(text="MASS: 0 kg", position=(-0.75, 0.10))  # Adjust the position as needed

    timer_label = Text(text="TIMER: 00:00", position=(-0.75, 0.05))  # Adjust the position as needed

    start_time = None  # Initialize the start time to None
   


initialize_ui()

# ===== Physics Setup =====

def setup_physics():
    global earth_body, rocket_body
    
    earth_position = [0, 0, 0]
    earth_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=earth_radius_game)
    earth_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=earth_shape, 
                                   basePosition=earth_position)
    
    rocket_max_velocity = 7500  # This is just an example value for Falcon 9. Adjust as needed

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


# Create the minimap camera
minimap_camera = Entity(
    parent=camera.ui,  # Attach to the UI for overlay
    model='quad',
    scale=(0.2, 0.2),  # Adjust the scale to control the size of the minimap
    position=(0.65, -0.4),  # Adjust the position of the minimap
    color=color.gray,
)

# Create a UI element to represent the player's position on the minimap
player_marker = Entity(
    parent=minimap_camera,
    model='quad',
    scale=0.02,
    color=color.red,
)

# Update the position of the player marker on the minimap
def update_minimap():
    player_marker.world_position = Vec3(
        rocket.entity.world_position.x * minimap_camera.scale_x,
        rocket.entity.world_position.y * minimap_camera.scale_y,
        0
    )



# Update function
def update():

    #update_minimap()

    rocket.update()
    earth.update()
    moon.update()
    game_camera.update()


    altitude = rocket.entity.world_position.y - earth_radius_game

    altitude = altitude / 1000 / scale_factor
    # Determine the current atmospheric layer
    if altitude <= 12:
        ratio = altitude / (troposphere_radius - earth_radius_game)
        current_color = lerp(troposphere_color, stratosphere_color, ratio)
    elif altitude <= 50:
        ratio = (altitude - (troposphere_radius - earth_radius_game)) / (stratosphere_radius - troposphere_radius)
        current_color = lerp(stratosphere_color, mesosphere_color, ratio)
    elif altitude <= 85:
        ratio = (altitude - (stratosphere_radius - earth_radius_game)) / (mesosphere_radius - stratosphere_radius)
        current_color = lerp(mesosphere_color, thermosphere_color, ratio)
    elif altitude <= 600:
        ratio = (altitude - (mesosphere_radius - earth_radius_game)) / (thermosphere_radius - mesosphere_radius)
        current_color = lerp(mesosphere_color, exosphere_color, ratio)
    else:
        current_color = exosphere_color


    if altitude <= 384400:
        p.stepSimulation()


    window.color = current_color

app.run()
