from ursina import *
from panda3d.core import LMatrix4f
from panda3d.core import ClockObject
from ursina.ursinastuff import *

import pybullet as p
import numpy as np

from scipy.spatial.transform import Rotation, Slerp

def slerp_scipy(start, end, t):
    rotations = Rotation.from_quat([start, end])
    slerp_obj = Slerp([0, 1], rotations)
    interpolated_rotation = slerp_obj(t)  # `t` should be within the range [0, 1]
    return interpolated_rotation.as_quat()[0]

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180)

def radians_to_degrees(radians):
    return radians * (180 / math.pi)

def euler_to_degrees(euler):
    return [radians_to_degrees(euler[0]), radians_to_degrees(euler[1]), radians_to_degrees(euler[2])]

def get_sign(number):
    return int(math.copysign(1, number))

# Increase the FPS limit, for example, set it to 120.
window.vsync = True

app = Ursina()

def set_frame_rate(value):
    # Set the clock mode to limited (i.e., limit the frame rate)
    globalClock.setMode(ClockObject.MLimited)
    
    # Set the desired frame rate
    globalClock.setFrameRate(int(value))

# Call the function with the desired frame rate, e.g., 120
set_frame_rate(1200)

# Start PyBullet
physics_client = p.connect(p.DIRECT)

p.setGravity(0, -9.81, 0)

# Ensure connection was successful.
if physics_client == -1:
    print("Failed to connect to pybullet.")
    exit()

#moon_scale_radius = 0.017828303850156088
#moon_scale_radius = 0.17828303850156088
scale_radius = 0.0001

thrust_multiplier = 100
#thrust_multiplier = 100
isp_multiplier = 1

earth_radius = 6371000 
moon_radius = 173710 
rocket_height = 42 
rocket_radius = 4 
moon_distance = 384400e3 

thrust_kN = 845 * thrust_multiplier #* moon_scale_radius  # Thrust of Merlin 1D engine in kN
total_propellant_mass = 523000 #* moon_scale_radius# in kg

# Define the mass of Earth in your game's scale
earth_mass = 5.972e24 #* moon_scale_radius  # Mass of Earth in kg
moon_mass = 7.342e22  #* moon_scale_radius # Mass of Moon in kg

# Falcon 9 dimensions
radius = 1.83
height = 55.7

# Falcon 9 dry mass (without fuel)
dry_mass = 26000 #* moon_scale_radius  # in kg

# Calculate the volume of the Falcon 9
falcon9_volume = math.pi * radius**2 * height

# Calculate the inferred average density
average_density = dry_mass / falcon9_volume


earth_radius_game = earth_radius * scale_radius
moon_distance_game = moon_distance * scale_radius
moon_radius_game = moon_radius * scale_radius

rocket_height_game = 200  * scale_radius
rocket_radius_game = 50 * scale_radius

earth_position = [0, 0, 0]
moon_position = [0, moon_distance, 0]

# Define the real-world distances for the atmospheric layers
troposphere_distance = 120000 
stratosphere_distance = 500000 
mesosphere_distance = 850000 
thermosphere_distance = 600000 

# Convert the real-world distances (now in km) to the game's scale
troposphere_distance_game = troposphere_distance * scale_radius
stratosphere_distance_game = stratosphere_distance * scale_radius
mesosphere_distance_game = mesosphere_distance * scale_radius
thermosphere_distance_game = thermosphere_distance * scale_radius

troposphere_color = color.rgb(0, 0, 255, 100)  # Blue with alpha
stratosphere_color = color.rgb(150, 150, 255, 40)
mesosphere_color = color.rgb(80, 80, 200, 30)
thermosphere_color = color.rgb(40, 40, 150, 20)
exosphere_color = color.rgb(0, 0, 0)

# Initial gimbal rotation angle (in degrees)
gimbal_rotation_angle = 0.0

# Calculate the radii of each layer based on the game's scale
troposphere_radius = earth_radius + troposphere_distance
stratosphere_radius = earth_radius + stratosphere_distance
mesosphere_radius = earth_radius + mesosphere_distance
thermosphere_radius = earth_radius + thermosphere_distance


# Create the Earth (Sphere)
earth_position = [0, 0, 0]  # Earth is at the origin now

# Define atmospheric drag constants
air_density = 1.225  # kg/m^3 (standard air density at sea level)
drag_coefficient = 0.47  # A typical drag coefficient for a streamlined object
frontal_area = math.pi * (rocket_radius**2)  # Using math.pi for the calculation

is_rocket_state = True
transition_speed = 2.0  # Adjust the transition speed as needed

# Given values
mixture_ratio = 2.56  # LOX:RP-1

density_LOX = 1141  # in kg/m^3
density_RP1 = 820   # in kg/m^3

# Calculate mass of LOX and RP-1
mass_LOX = total_propellant_mass / (1 + 1/mixture_ratio)
mass_RP1 = total_propellant_mass - mass_LOX

# Calculate volume of LOX and RP-1
volume_LOX = mass_LOX / density_LOX
volume_RP1 = mass_RP1 / density_RP1

total_rocket_mass = dry_mass + total_propellant_mass

# Constants
G = 6.674e-11
#M = 5.972e24
total_distance = 2000 #* moon_scale_radius
r_initial = moon_radius + total_distance 

class Earth:
    def __init__(self):
        self.entity = Entity(model='sphere', color=color.rgb(139, 69, 19), scale=earth_radius_game)
        self.rotation_speed = 10

    def update(self):
        pass
        #self.entity.rotation_y += time.dt * self.rotation_speed

class Rocket:
    def __init__(self, body):
        self.entity = Entity(scale_y=rocket_height_game, scale_x=rocket_radius_game, scale_z=rocket_radius_game, color=color.azure, model='cube')

        self.rocket = Entity(parent=self.entity, model='soyuz-fg.glb', scale_y=rocket_height_game, scale_x=rocket_height_game, scale_z=rocket_height_game)
        self.rocket.visible = False

        self.rocket.position = (0,-rocket_height_game * 10, 0)
        #self.rocket.scale *= 0.00005

        self.entity.world_position = Vec3(0, earth_radius_game + rocket_height_game / 2, 0)

        self.rocket_body = body

        self.rocket_collision_shape = Entity(parent=self.entity, model='cube', color=color.red)
        self.rocket_collision_shape.scale_x = rocket_radius_game
        self.rocket_collision_shape.scale_y = rocket_height_game
        self.rocket_collision_shape.scale_z = rocket_radius_game
        self.rocket_collision_shape.visible =  False

        self.gimbal_rotation_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0
        self.roll_angle = 0.0

        scale = 10

        self.gimbal = Entity(parent=self.entity, model='cube', color=color.black, scale=(scale, scale, scale))

        self.gimbal.y = (self.entity.scale.y / 4 / self.rocket.scale.y) * -0.1

        self.gimbal.visible = False

        stage_mass = total_rocket_mass - total_propellant_mass
        # Defining stage masses
        self.first_stage_mass = stage_mass * 0.4  # Example mass distribution
        self.second_stage_mass = stage_mass * 0.35
        self.third_stage_mass = stage_mass * 0.25
        
        # Starting with all stages attached
        self.current_mass = total_rocket_mass
        
        # Representing stages with simple shapes (e.g., cylinders)
        # Set the parent at the time of creation.

        scale_mult = 250
        self.first_stage = Entity(parent=self.entity, scale_y=rocket_radius_game * scale_mult / 4, scale_x=rocket_radius_game * scale_mult, scale_z=rocket_radius_game * scale_mult, color=color.black, model='cube')
        self.first_stage.position = Vec3(0, -0.4, 0)
        
        self.second_stage = Entity(parent=self.entity, scale_y=rocket_radius_game * scale_mult / 4, scale_x=rocket_radius_game * scale_mult, scale_z=rocket_radius_game * scale_mult, color=color.red, model='cube')
       
        self.second_stage.position = Vec3(0, 0, 0)

        self.first_stage.visible = True
        self.second_stage.visible = True
        # Position the second_stage just above the top of self.entity. 
        # If the height of self.entity is based on its y-scale, then:
        # self.second_stage.y = 0 

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


        self.traveled_distance = 0
        self.current_fuel_mass = total_propellant_mass

        self.current_stage = 1  # Starting stage

        self.frame_counter = 0  # Counting frames since the last UI update
        self.previous_velocity = (0, 0, 0)  # Assuming it starts at rest
        self.thrust_direction = Vec3(0, 1, 0)  # By default, thrust upwards
        self.valve_multiplier = 1.0  # 100% open, can vary between 0 and 1

        # Rocket thrust variables
        self.thrust_force = thrust_kN * 1000

        #self.isp = 311  # specific impulse in seconds (for an RP-1/LOX engine)

        self.isp = 311 * thrust_multiplier * isp_multiplier

        self.num_engines = 9
        self.altitude = 0
        self.rotated = False
        self.gravity = 0

        #self.jettison_first_stage()
        #self.jettison_second_stage()
        self.mode = 'launch'

        self.modes = ['launch', 'update']

    def select_mode(self):
        if held_keys['1']:  # V for reducing valve opening
            self.mode = self.modes[0]
        if held_keys['2']:  # B for increasing valve opening
            self.mode = self.modes[1]

    def discharge_jettison(self):
        # Jettison stages with number keys
        if held_keys['1']:
            self.jettison_first_stage()
        if held_keys['2']:
            self.jettison_second_stage()

    def adjust_valve_opening(self):
        # Use two keys to adjust the valve opening
        if held_keys['c']:  # V for reducing valve opening
            self.valve_multiplier = 0.01  # decrement by 1% (adjust as needed)
            self.valve_multiplier = max(0, self.valve_multiplier)  # limit to a minimum of 0
        if held_keys['v']:  # B for increasing valve opening
            self.valve_multiplier = 1  # increment by 1% (adjust as needed)
            self.valve_multiplier = min(1, self.valve_multiplier)  # limit to a maximum of 1

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

    def get_rocket_orientation(self):
        _, orientation_quaternion = p.getBasePositionAndOrientation(self.rocket_body)
        return orientation_quaternion

    def quaternion_to_euler(self, quaternion):
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

    def _update_rocket_transform(self):
        pos, physics_rotation = p.getBasePositionAndOrientation(rocket.rocket_body)
        new_euler = p.getEulerFromQuaternion(physics_rotation)
        rotation = euler_to_degrees(new_euler)
        rotation = [round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)]

        self.entity.position = Vec3(pos[0] * scale_radius, pos[1] * scale_radius, pos[2] * scale_radius)
        self.entity.rotation =  rotation

    def update(self):
        self.select_mode()
        self.gravity = self._apply_gravity()
        effective_gravity = self.compute_effective_gravitational_acceleration()
        self._adjust_gimbal_rotation()
        self._calculate_thrust_direction()

        self._consume_fuel_and_apply_thrust()
        self._apply_drag_force()
        self._update_altitude()
        self.adjust_valve_opening()
        self.discharge_jettison()
        self._update_rocket_transform()

        self.frame_counter += 1

        # Update the UI elements every 10 frames (or whatever number you prefer)
        if self.frame_counter % 6 == 0:
            # Determine the current atmospheric layer
            altitude = self.altitude
            if altitude <= troposphere_distance:
                atmospheric_layer = "Troposphere"
            elif altitude <= stratosphere_distance:
                atmospheric_layer = "Stratosphere"
            elif altitude <= mesosphere_distance:
                atmospheric_layer = "Mesosphere"
            elif altitude <= thermosphere_distance:
                atmospheric_layer = "Thermosphere"
            else:
                atmospheric_layer = "Exosphere"

            velocity = p.getBaseVelocity(self.rocket_body)[0]  # Assuming this returns a velocity vector

            dt = time.dt
            speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2) # Calculate the magnitude of the velocity vector

            distance_traveled = speed * dt

            self.traveled_distance += distance_traveled 

            speed_label.text = f"SPEED: {speed:.2f} m/s" 

            # Compute acceleration magnitude
            acceleration = math.sqrt((velocity[0] - self.previous_velocity[0])**2 + 
                                    (velocity[1] - self.previous_velocity[1])**2 + 
                                    (velocity[2] - self.previous_velocity[2])**2) / dt


            acceleration_label.text = f"ACCEL: {acceleration:.2f} m/s^2"

            gravity_label.text = f"G: {effective_gravity:.002f} m/s^2"
            
            self.previous_velocity = velocity  # Store current velocity for next frame


            atmospheric_layer_label.text = f"LAYER: {atmospheric_layer}"


            fuel_percentage = (self.current_fuel_mass / total_propellant_mass) * 100  # Calculate the percentage of fuel remaining

            altitude_label.text = f"ALTITUDE: {altitude / 1000:.2f} KM"  # Rounded to two decimal places for display
            distance_label.text = f"DISTANCE: {self.traveled_distance / 1000:.2f} KM"  # Rounded to two decimal places for display

            fuel_label.text = f"FUEL: {int(fuel_percentage):.2f}%"  # Display the fuel percentage
        
            # To calculate the speed, we need the change in position divided by time.
            current_stage_label.text = f"STAGE: {self.current_stage}"

            info = p.getDynamicsInfo(self.rocket_body, -1)
            current_mass = info[0]
            mass_label.text = f"MASS: {current_mass:.2f} kg"

    def _adjust_gimbal_rotation(self):
        global held_keys

        angle_change = 0.001 * time.dt
        yaw_change = 0
        pitch_change = 0
        roll_change = 0

        if held_keys['down arrow']:
            yaw_change += angle_change
        elif held_keys['up arrow']:
            yaw_change -= angle_change

        if held_keys['right arrow']:
            pitch_change += angle_change
        elif held_keys['left arrow']:
            pitch_change -= angle_change

        if held_keys['q']:
            roll_change += angle_change
        elif held_keys['e']:
            roll_change -= angle_change

        self.pitch_angle += pitch_change
        self.yaw_angle += yaw_change
        self.roll_angle += roll_change

        degree_clamp = 1
        # Limit gimbal rotation angles within a range
        self.pitch_angle = clamp(self.pitch_angle, -degree_clamp, degree_clamp)
        self.yaw_angle = clamp(self.yaw_angle, -degree_clamp, degree_clamp)
        self.roll_angle = clamp(self.roll_angle, -degree_clamp, degree_clamp)

    def _calculate_thrust_direction(self):
        angle_change = any([held_keys[key] for key in ['down arrow', 'up arrow', 'right arrow', 'left arrow', 'q', 'e']])


        if angle_change:
            # Compute thrust direction based on user inputs
            pitch_matrix = LMatrix4f.rotateMat(self.pitch_angle, Vec3(1, 0, 0))
            yaw_matrix = LMatrix4f.rotateMat(self.yaw_angle, Vec3(0, 0, 1))
            roll_matrix = LMatrix4f.rotateMat(self.roll_angle, Vec3(0, 1, 0))
            
            rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix
            self.thrust_direction = rotation_matrix.xformVec(Vec3(0, 1, 0))

        else:
            # Compute thrust direction based on rocket's current orientation
            pos, physics_rotation = p.getBasePositionAndOrientation(rocket.rocket_body)
            new_euler = p.getEulerFromQuaternion(physics_rotation)
            computed_euler = euler_to_degrees(new_euler)
            
            pitch_matrix = LMatrix4f.rotateMat(computed_euler[0], Vec3(1, 0, 0))
            yaw_matrix = LMatrix4f.rotateMat(computed_euler[1], Vec3(0, 0, 1))
            roll_matrix = LMatrix4f.rotateMat(computed_euler[2], Vec3(0, 1, 0))
            
            rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix
            self.thrust_direction = rotation_matrix.xformVec(Vec3(0, 1, 0))
    
        self.thrust_direction.normalize()
        print(self.thrust_direction)


    def decompose_matrix_to_euler(self, mat):
        # Extract the rotation matrix components
        m00, m01, m02 = mat[0][0], mat[0][1], mat[0][2]
        m10, m11, m12 = mat[1][0], mat[1][1], mat[1][2]
        m20, m21, m22 = mat[2][0], mat[2][1], mat[2][2]
        
        # Calculate yaw, pitch, and roll in radians
        yaw = math.atan2(m10, m00)
        pitch = math.asin(-m20)
        roll = math.atan2(m21, m22)
        
        return (roll, yaw, pitch) 
               
                
    def _recalculate_thrust_direction(self):
        pass

    def gravitational_force(self, object_position, celestial_body_position, m_rocket, m_body):
        direction = Vec3(*celestial_body_position) - Vec3(*object_position)

        # Distance between rocket and celestial body
        r = direction.length()

        # Magnitude of the gravitational force on the rocket
        force_magnitude = (G * m_rocket * m_body) / (r**2)

        # Gravitational force vector (magnitude * direction)
        force_vector = direction.normalized() * force_magnitude

        # Computing gravitational acceleration g for the rocket at its position
        #g = (G * m_body) / (r**2)

        g = force_vector.y / self.current_mass

        return force_vector, g
    
    def compute_gravitational_force(self, rocket_mass, g):
        """
        Compute the gravitational force acting on the rocket given its mass and the gravitational acceleration.
        :param rocket_mass: Mass of the rocket.
        :param g: Gravitational acceleration as a vector (g_x, g_y, g_z).
        :return: Gravitational force as a vector (F_x, F_y, F_z).
        """
        F_x = rocket_mass * g
        F_y = rocket_mass * g
        F_z = rocket_mass * g

        return Vec3(F_x, F_y, F_z)

    
    def _apply_gravity(self):
        rocket_position, _ = p.getBasePositionAndOrientation(self.rocket_body)

        # Earth's data
        force_earth, eg = self.gravitational_force(rocket_position, earth_position, self.current_mass, earth_mass)

        # Moon's data
        # Note: You'll need to update moon_position dynamically if the Moon orbits the Earth in your simulation.
        # For a static approximation, you can use a fixed position relative to Earth.
        force_moon, mg = self.gravitational_force(rocket_position, Vec3(-moon_distance), self.current_mass, moon_mass)

        # Total gravitational force on the rocket
        total_gravity_force = force_earth - force_moon

        g = total_gravity_force / self.current_mass

        

        force_application_point = [0, 0, 0]

        rg = self.compute_gravitational_force(g.y, self.current_mass)
        #p.applyExternalForce(self.rocket_body, -1, [0, rg.y, 0], force_application_point, p.WORLD_FRAME)
            
        return g
    
    def compute_L1(self):
        # Assuming G and the masses of Earth and Moon are already defined
        # and that moon_distance is the distance from Earth to Moon

        # Setting gravitational forces of Earth and Moon equal to each other
        r = moon_distance * (moon_mass / (3 * earth_mass))**(1/3)

        return r

    
    def compute_effective_gravitational_acceleration(self):
        rocket_position, _ = p.getBasePositionAndOrientation(self.rocket_body)
        distance_to_earth = (Vec3(*rocket_position) - Vec3(*earth_position)).length()
        
        # Distance to the moon calculation remains the same
        distance_to_moon = Vec3(moon_distance - moon_radius - rocket_position[1] - rocket_height / 2).length()

        # Get L1 distance
        L1 = self.compute_L1()

        distance_from_rocket_to_L1 = distance_to_moon - L1

        # Calculate the gravitational forces
        _, eg = self.gravitational_force(rocket_position, earth_position, self.current_mass, earth_mass)
        _, mg = self.gravitational_force(Vec3((rocket_position[1] + rocket_height / 2)), 
                                         Vec3(-(moon_distance)), 
                                         self.current_mass, 
                                         moon_mass)

        # Linear interpolation of forces based on distances
        ratio_to_L1 = distance_from_rocket_to_L1 / L1

        mg = mg * 1e6

        distance_from_rocket_to_L1_2 = L1 - distance_to_moon
        ratio_to_L1_2 = distance_from_rocket_to_L1_2 / L1 

        ratio_to_L1_2 = max(0, min(1.0, ratio_to_L1_2))

        if distance_to_moon < L1:
            effective_gravity = mg * ratio_to_L1_2
        else:
            distance_from_rocket_to_L1 = abs(distance_from_rocket_to_L1)
            ratio_to_L1 = max(0, min(1, ratio_to_L1))
            effective_gravity = ratio_to_L1 * eg
 
        return effective_gravity

    def debug_gravity(self, rocket_position, celestial_body_position, m_body):
        direction = Vec3(celestial_body_position) - Vec3(rocket_position)
        r = direction.length()
        g = (G * m_body) / (r**2)
        return g, r


    
    def _update_mass(self):
        # Calculate the current total mass of the rocket (initial mass + remaining fuel)
        current_total_mass = self.current_mass

        # Update the bullet body mass
        p.changeDynamics(self.rocket_body, -1, mass=current_total_mass, linearDamping=0.0, angularDamping=0.0, maxJointVelocity = 1500000)
    
    def acceleration_to_thrust(self, acceleration):
        # Implement conversion of acceleration to thrust if needed...
        # For simplicity, let's assume thrust = acceleration * mass (Newtons's second law)
        return acceleration * self.current_mass
    
    def required_thrust(self, u, v, s, g_moon):  # g_moon is Moon's gravitational acceleration in m/s^2
        """
        Calculate the required thrust to change velocity from u to v over distance s in Moon's gravity field.
        """
        a_required = (v**2 - u**2) / (2 * s)
        thrust = a_required + g_moon  # Total acceleration required is the sum of Moon's gravity and the thrust to achieve desired velocity change.
        return thrust

    def required_accel(self, u, v, s):  # g_moon is Moon's gravitational acceleration in m/s^2
        """
        Calculate the required thrust to change velocity from u to v over distance s in Moon's gravity field.
        """
        a_required = (v**2 - u**2) / (2 * s)
        return a_required
    
    def compute_thrust_v2(self):
        MAX_THRUST = self.thrust_force * self.num_engines

        pos, _ = p.getBasePositionAndOrientation(self.rocket_body)
        velocity = p.getBaseVelocity(self.rocket_body)[0]
        v = -math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        dist_to_moon = Vec3(moon_distance - moon_radius - pos[1] - rocket_height / 2)

        remaining_distance = dist_to_moon.y

        g = self.gravity

        gravitational_force = g.y

        a_gravity = gravitational_force / self.current_mass
        required_deceleration = 0

        if remaining_distance > 0:
            required_deceleration = v**2 / (2 * remaining_distance)

        # Calculate the actual thrust required to get the required deceleration
        thrust_needed = (required_deceleration + a_gravity) * self.current_mass

        # Adjust thrust dynamically, but don't exceed MAX_THRUST
        actual_thrust = min(thrust_needed, MAX_THRUST)
        
        return actual_thrust


    def compute_thrust_v3(self):
        MAX_THRUST = self.thrust_force * self.num_engines

        pos, _ = p.getBasePositionAndOrientation(self.rocket_body)
        velocity = p.getBaseVelocity(self.rocket_body)[0]
        v = -math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        dist_to_moon = Vec3(moon_distance - moon_radius - pos[1] - rocket_height / 2)

        remaining_distance = dist_to_moon.y

        g = self.gravity

        

        gravitational_force = g.y

        a_gravity = gravitational_force / self.current_mass
        required_deceleration = 0

        if remaining_distance > 0:
            required_deceleration = v**2 / (2 * remaining_distance)

        # Calculate the actual thrust required to get the required deceleration
        thrust_needed = (required_deceleration + a_gravity) * self.current_mass
        actual_thrust = min(thrust_needed, MAX_THRUST)
        
        return actual_thrust

    def compute_thrust(self):
        global at_moon
        pos, _ = p.getBasePositionAndOrientation(self.rocket_body)

        dist_to_moon = Vec3(moon_distance - moon_radius - pos[1] - rocket_height / 2)

        if dist_to_moon.y < r_initial:
            effective_thrust = -self.compute_thrust_v3()
        else:
            effective_thrust = self.thrust_force * self.num_engines

        effective_thrust = effective_thrust * self.valve_multiplier
        
        return effective_thrust

    def _consume_fuel_and_apply_thrust(self):
        global held_keys, oxygen_level, total_rocket_mass
        thrust_magnitude = self.compute_thrust()

        global start_time

        if ((self.current_fuel_mass > 0) and held_keys['space']):
            thrust_force_vector = self.thrust_direction * thrust_magnitude

            #force_application_point = [0, -rocket_height / 2, 0]
            force_application_point = [0, 0, 0]
            p.applyExternalForce(self.rocket_body, -1, thrust_force_vector, force_application_point, p.WORLD_FRAME)
            
            # Apply thrust force to the rocket's physics body
            g = self.gravity 
            g0 = -g.y  # standard gravity in m/s^2
            ve = self.isp * g0
            # Calculate mass flow rate
            self.mass_flow_rate = math.fabs(thrust_force_vector.y) / ve

            fuel_consumed_per_update = self.mass_flow_rate * time.dt

            fuel_fraction_left = self.current_fuel_mass / total_propellant_mass # fraction of initial fuel left
            oxygen_level.scale_x = fuel_fraction_left

            # Reduce remaining fuel

            if self.current_fuel_mass > 0:
                self.current_mass -= fuel_consumed_per_update
            self.current_fuel_mass -= fuel_consumed_per_update  # Decrease current fuel mass

            if start_time is None:
                start_time = time.time()  # Record the start time when space is first pressed

        if start_time:
            elapsed_time = time.time() - start_time
            minutes, seconds = divmod(int(elapsed_time), 60)
            timer_label.text = f"TIMER: {minutes:02}:{seconds:02}"

        self._update_mass()

    def _apply_drag_force(self):
        pass
        # Calculate drag force
        # velocity = p.getBaseVelocity(self.rocket_body)[0]
        # velocity_magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        # drag_force = 0.5 * air_density * velocity_magnitude**2 * drag_coefficient * frontal_area

        # Apply drag force in the opposite direction of velocity
        # drag_force_direction = (-velocity[0] / velocity_magnitude, -velocity[1] / velocity_magnitude, -velocity[2] / velocity_magnitude) if velocity_magnitude > 0 else (0, 0, 0)
        # drag_force_vec = (drag_force * drag_force_direction[0], drag_force * drag_force_direction[1], drag_force * drag_force_direction[2])

        # rocket_position, _ = p.getBasePositionAndOrientation(self.rocket_body)

        #p.applyExternalForce(self.rocket_body, -1, drag_force_vec, rocket_position, p.LINK_FRAME)
        
    def compute_altitude(self, rocket_pos, earth_position, earth_radius):
        # Compute the difference in position between the rocket and the center of the Earth
        dx = rocket_pos[0] - earth_position[0]
        dy = rocket_pos[1] - earth_position[1]
        dz = rocket_pos[2] - earth_position[2]

        # Compute the distance between the rocket and the center of the Earth
        distance_to_center = math.sqrt(dx**2 + dy**2 + dz**2)

        # Compute altitude by subtracting the Earth's radius from the distance to the center
        altitude = distance_to_center - earth_radius

        return altitude

    def _update_altitude(self):
        rocket_pos, _ = p.getBasePositionAndOrientation(self.rocket_body)
        self.altitude = self.compute_altitude(rocket_pos, earth_position, earth_radius)


class Moon:
    def __init__(self):
        pass
        #self.entity = Entity(model='sphere', color=color.gray, scale=moon_radius_game)
        #self.entity.world_position = earth.entity.world_position + Vec3(0, moon_distance_game, 0)

    def update(self):
        pass

class GameCamera:
    def __init__(self):
        self.entity = camera
        self.original_rotation = self.entity.rotation
        self.is_rocket_state = True  # default to rocket state

    def update(self):
        if held_keys['o']:
            #self.entity.fov = 10
            self.is_rocket_state = True

        # if held_keys['p']:
        #     self.entity.fov = 60
        #     self.is_rocket_state = False

        self._update_position_and_rotation()

    def _update_position_and_rotation(self):
        if self.is_rocket_state:
            self.entity.clip_plane_near = 0.001
            self.entity.clip_plane_far = 10000
            self.entity.fov = 15

            target_position = rocket.entity.world_position + Vec3(0.4, 0, 0)

            #target_rotation = self.original_rotation
            self._set_camera(target_position)
            self.entity.look_at(rocket.entity)

    def _set_camera(self, position):
        self.entity.position = position
        #self.entity.rotation = rotation

def initialize_ui():
    global oxygen_bar, oxygen_level, altitude_label, distance_label, fuel_label, speed_label, current_stage_label, atmospheric_layer_label, mass_label, timer_label, start_time, acceleration_label, orientation_label, gravity_label

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
    
    orientation_label = Text(label="", position=(-0.75, 0.0), background=True)

    gravity_label = Text(text="GRAVITY: 0 m/s^2", position=(-0.75, -0.05))  # Gravity in terms of acceleration

initialize_ui()

# ===== Physics Setup =====

def setup_physics():
    global earth_body, moon_body, rocket_body
        
    # Create the Earth
    earth_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=earth_radius)
    earth_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=earth_shape, basePosition=earth_position)

    # Create the Earth
    moon_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=moon_radius)
    moon_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=moon_shape, basePosition=moon_position)


    # Calculate the rocket's initial position at the edge of the Earth's surface
    rocket_start_position = [0, earth_radius + rocket_height / 2, 0]
    shape_offset = [0, -rocket_height, 0]

    # Create the rocket
    rocket_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[rocket_radius, rocket_height / 2, rocket_radius])
    rocket_body = p.createMultiBody(baseMass=total_rocket_mass, baseCollisionShapeIndex=rocket_shape, basePosition=rocket_start_position, baseInertialFramePosition=shape_offset)

setup_physics()

def setup_environment():
    # Number of pyramids to populate
    num_pyramids = 1

    # Create a function to get a random point on a sphere's surface
    def random_sphere_point(radius):
        theta = 2 * math.pi * random.random()
        phi = math.acos(2 * random.random() - 1)
        x = radius * math.sin(phi) * math.cos(theta)
        y = radius * math.sin(phi) * math.sin(theta)
        z = radius * math.cos(phi)
        return (x, y, z)


    class CustomBox(Entity):
        def __init__(self, **kwargs):
            super().__init__()

            # Each side of the cube is a quad with the texture applied
            self.sides = []
            for i in range(6):  # 6 sides for a cube
                side = Entity(parent=self, model='quad', texture='camo.png')
                if i == 0:  # front
                    side.rotation_y = 0
                    side.z = 0.5
                if i == 1:  # right
                    side.rotation_y = 90
                    side.x = 0.5
                if i == 2:  # back
                    side.rotation_y = 180
                    side.z = -0.5
                if i == 3:  # left
                    side.rotation_y = -90
                    side.x = -0.5
                if i == 4:  # top
                    side.rotation_x = 90
                    side.y = 0.5
                if i == 5:  # bottom
                    side.rotation_x = -90
                    side.y = -0.5

                self.sides.append(side)

            for key, value in kwargs.items():
                setattr(self, key, value)

    # Create pyramids at random points on the sphere's surface
    for _ in range(num_pyramids):
        position = random_sphere_point(earth_radius_game)
        pyramid = CustomBox()
        pyramid.position = position
        pyramid.scale = 1
        pyramid.look_at(Vec3(0,0,0))  # Make the pyramid point towards the sphere's center


setup_environment()

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

def moon_landing():
    velocity = p.getBaseVelocity(self.rocket_body)[0]  # Assuming this returns a velocity vector

    dt = time.dt
    speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2) # Calculate the magnitude of the velocity vector

    print("S: " + str(speed))
    game_camera.update()

progress = 1
def rotation_test():
    pos, rot = p.getBasePositionAndOrientation(rocket.rocket_body)

    # 1. Convert the Euler angles to a quaternion.
    new_rotation_quaternion = p.getQuaternionFromEuler([0, 0, degrees_to_radians(180)])

    # 2. Multiply the current quaternion rot by the newly obtained quaternion.
    combined_quaternion = p.multiplyTransforms([0, 0, 0], rot, [0, 0, 0], new_rotation_quaternion)[1]

    # 3. Convert the combined Euler angles back to a quaternion
    new_euler = p.getEulerFromQuaternion(combined_quaternion)

    rocket.entity.position = Vec3(pos[0] * scale_radius, pos[1] * scale_radius, pos[2] * scale_radius)
    rocket.entity.rotation =  euler_to_degrees(new_euler)

    #rocket.entity.rotation = [0, 90, 90]

def super_update(loop_time):
    for x in range (0, loop_time):
        p.stepSimulation()
        
        #update_minimap()

        rocket.update()
        earth.update()
        moon.update()
        game_camera.update()

        altitude = rocket.altitude

        # Determine the current atmospheric layer
        if altitude <= troposphere_distance:
            ratio = altitude / (troposphere_radius - earth_radius)
            current_color = lerp(troposphere_color, stratosphere_color, ratio)
        elif altitude <= stratosphere_distance:
            ratio = (altitude - (troposphere_radius - earth_radius)) / (stratosphere_radius - troposphere_radius)
            current_color = lerp(stratosphere_color, mesosphere_color, ratio)
        elif altitude <= mesosphere_distance:
            ratio = (altitude - (stratosphere_radius - earth_radius)) / (mesosphere_radius - stratosphere_radius)
            current_color = lerp(mesosphere_color, thermosphere_color, ratio)
        elif altitude <= thermosphere_distance:
            ratio = (altitude - (mesosphere_radius - earth_radius)) / (thermosphere_radius - mesosphere_radius)
            current_color = lerp(mesosphere_color, exosphere_color, ratio)
        else:
            current_color = exosphere_color
        
        window.color = current_color

def super_late_update():
    rocket.late_update()

at_moon = False

class Terra(Entity):
    def __init__(self, **kwargs):
        super().__init__()
        self.accumulated_time = 0
        self.fixed_update_interval = 1/20  # 60 times per second

    # Update function
    def update(self):
        self.accumulated_time += time.dt

        global at_moon
        if at_moon:
            return
        super_update(1)

        # while self.accumulated_time >= self.fixed_update_interval:
        #     self.accumulated_time -= self.fixed_update_interval

    def fixed_update(self):
        super_late_update()
        

terra = Terra()
app.run()