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
window.vsync = False


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

p.setGravity(0, 0, 0)

# Ensure connection was successful.
if physics_client == -1:
    print("Failed to connect to pybullet.")
    exit()

scale_radius = 0.0001

earth_radius = 6371000
moon_radius = 173710
rocket_height = 42
rocket_radius = 4
moon_distance = (228440000) * 0.03

earth_radius_game = earth_radius * scale_radius
moon_distance_game = moon_distance * scale_radius
moon_radius_game = moon_radius * scale_radius

rocket_height_game = 200  * scale_radius
rocket_radius_game = 50 * scale_radius

earth_position = [0, 0, 0]
moon_position = [0, moon_distance, 0]


# Define the real-world distances for the atmospheric layers
troposphere_distance = 120000 * 0.022 # in  m
stratosphere_distance = 500000 * 0.022  # in  m
mesosphere_distance = 850000 * 0.022 # in  m
thermosphere_distance = 600000 * 0.022  # in m

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

# Define the mass of Earth in your game's scale
earth_mass = 5.972e24  # Mass of Earth in kg
#moon_mass = 7.342e22  # Mass of Moon in kg
moon_mass = 7.342e22 * 0.022  # Mass of Moon in kg

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

# Falcon 9 dimensions
radius = 1.83  # in meters
height = 55.7  # in meters

# Falcon 9 dry mass (without fuel)
dry_mass = 26000  # in kg

# Calculate the volume of the Falcon 9
falcon9_volume = math.pi * radius**2 * height

# Calculate the inferred average density
average_density = dry_mass / falcon9_volume

# Given values
total_propellant_mass = 523000  # in kg
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
total_distance = 2000
r_initial = moon_radius + total_distance 

class Earth:
    def __init__(self):
        self.entity = Entity(model='sphere', color=color.rgb(139, 69, 19), scale=earth_radius_game)
        self.rotation_speed = 10

    def update(self):
        self.entity.rotation_y += time.dt * self.rotation_speed


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
        thrust_kN = 845  # Thrust of Merlin 1D engine in kN
        self.thrust_force = thrust_kN * 1000  # Convert kN to N

        self.isp = 311  # specific impulse in seconds (for an RP-1/LOX engine)

        self.num_engines = 9
        self.altitude = 0
        self.rotated = False
        self.lagrangian = False
        self.landing = False
        self.moonizage_distance = 0

    def adjust_valve_opening(self):
        # Use two keys to adjust the valve opening
        if held_keys['v']:  # V for reducing valve opening
            self.valve_multiplier = 0.01  # decrement by 1% (adjust as needed)
            self.valve_multiplier = max(0, self.valve_multiplier)  # limit to a minimum of 0
        if held_keys['b']:  # B for increasing valve opening
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

    def _update_rocket_visualization(self):
        euler_orientation = self.get_rocket_orientation()
        # Set the rotation of the visual rocket model based on PyBullet's physics simulation
        # Note: Ursina uses degrees for rotation, and PyBullet returns radians.
        # Therefore, you need to convert from radians to degrees.
        #self.rocket.rotation = self.quaternion_to_euler(euler_orientation)

        #print(self.rocket.rotation)

    def update(self):
        self._adjust_gimbal_rotation()
        self._calculate_thrust_direction()
        total_gravity = self._apply_gravity()
        self._consume_fuel_and_apply_thrust()
        self._apply_drag_force()
        self._update_position()
        self._update_info()
        self._update_rocket_visualization()
        self.adjust_valve_opening()

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

            gravity_label.text = f"G: {total_gravity.y:.2f} m/s^2"
            gravity_label.visible = False
            
            self.previous_velocity = velocity  # Store current velocity for next frame


            atmospheric_layer_label.text = f"LAYER: {atmospheric_layer}"


            fuel_percentage = (self.current_fuel_mass / total_propellant_mass) * 100  # Calculate the percentage of fuel remaining

            altitude_label.text = f"ALTITUDE: {altitude:.2f} M"  # Rounded to two decimal places for display
            distance_label.text = f"DISTANCE: {self.traveled_distance:.2f} M"  # Rounded to two decimal places for display

            fuel_label.text = f"FUEL: {int(fuel_percentage):.2f}%"  # Display the fuel percentage
        
            # To calculate the speed, we need the change in position divided by time.
            current_stage_label.text = f"STAGE: {self.current_stage}"

            info = p.getDynamicsInfo(self.rocket_body, -1)
            current_mass = info[0]
            mass_label.text = f"MASS: {current_mass:.2f} kg"

            # roll, pitch, yaw = self.get_rocket_orientation()
            # orientation_label.text = f"R: {math.degrees(roll):.2f}°, P: {math.degrees(pitch):.2f}°, Y: {math.degrees(yaw):.2f}°"

    def _adjust_gimbal_rotation(self):
        global held_keys

        angle_change = 1

        if held_keys['left arrow']:
            self.yaw_angle += angle_change
        elif held_keys['right arrow']:
            self.yaw_angle -= angle_change
        elif self.yaw_angle > 0:
            self.yaw_angle -= angle_change
        elif self.yaw_angle < 0:
            self.yaw_angle += angle_change

        if held_keys['up arrow']:
            self.pitch_angle += angle_change
        elif held_keys['down arrow']:
            self.pitch_angle -= angle_change
        elif self.pitch_angle > 0:
            self.pitch_angle -= angle_change
        elif self.pitch_angle < 0:
            self.pitch_angle += angle_change

        if held_keys['q']:
            self.roll_angle += angle_change
        elif held_keys['e']:
            self.roll_angle -= angle_change
        elif self.roll_angle > 0:
            self.roll_angle -= angle_change
        elif self.roll_angle < 0:
            self.roll_angle += angle_change

        # Jettison stages with number keys
        if held_keys['1']:
            self.jettison_first_stage()
        if held_keys['2']:
            self.jettison_second_stage()

        if self.yaw_angle != 0 or self.pitch_angle != 0:
            self.valve_multiplier = 0.0000001
        else:
            self.valve_multiplier = 1

        degree_clamp = 5
        # Limit gimbal rotation angles within a range
        self.pitch_angle = clamp(self.pitch_angle, -degree_clamp, degree_clamp)
        self.yaw_angle = clamp(self.yaw_angle, -degree_clamp, degree_clamp)
        self.roll_angle = clamp(self.roll_angle, -degree_clamp, degree_clamp)

    def _calculate_thrust_direction(self):
        pitch_radians = math.radians(self.pitch_angle)
        yaw_radians = math.radians(self.yaw_angle)
        roll_radians = math.radians(self.roll_angle)

        yaw_matrix = LMatrix4f.rotateMat(yaw_radians, Vec3(0, 0, 1))  # Using Vec3 to represent the forward direction

        pitch_matrix = LMatrix4f.rotateMat(pitch_radians, Vec3(1, 0, 0))
        
        rotation_matrix = yaw_matrix * pitch_matrix
        self.thrust_direction = rotation_matrix.xformVec(Vec3(0, 1, 0))
        self.thrust_direction.normalize()

    def gravitational_force(self, celestial_body_position, r_body, m_body):
        G = 6.67430e-11

        rocket_position, _ = p.getBasePositionAndOrientation(self.rocket_body)

        direction = Vec3(*celestial_body_position) - Vec3(*rocket_position)

        # Distance between rocket and celestial body
        r = direction.length()

        # Magnitude of the gravitational force
        # Magnitude of the gravitational force
        force_magnitude = (G * r_body * m_body) / (r**2)

        # Gravitational force vector (magnitude * direction)
        force_vector = direction.normalized() * force_magnitude

        # Computing gravitational acceleration g for the rocket at its position
        g = force_magnitude / r_body

        return force_vector, g

    
    def _apply_gravity(self):
        # Earth's data
        force_earth, _ = self.gravitational_force(earth_position, self.current_mass, earth_mass)

        # Moon's data
        # Note: You'll need to update moon_position dynamically if the Moon orbits the Earth in your simulation.
        # For a static approximation, you can use a fixed position relative to Earth.
        force_moon, _ = self.gravitational_force(moon_position, self.current_mass, moon_mass)

        # Total gravitational force on the rocket
        total_gravity_force = force_earth + force_moon

        g = total_gravity_force / self.current_mass

        # Apply this force to the rocket's physics body. 
        # p.applyExternalForce(objectUniqueId=self.rocket_body, 
        #                     linkIndex=-1, 
        #                     forceObj=[g.x, g.y, g.z], 
        #                     posObj=[0, 0, 0], 
        #                     flags=p.WORLD_FRAME)
        
        return g

    def _update_mass(self):
        # Calculate the current total mass of the rocket (initial mass + remaining fuel)
        current_total_mass = self.current_mass

        # Update the bullet body mass
        p.changeDynamics(self.rocket_body, -1, mass=current_total_mass, linearDamping=0.0, angularDamping=0.0, maxJointVelocity = 1500)
    
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
        dist_to_moon = Vec3(moon_distance - moon_radius - pos[1] + rocket_height / 2)

        remaining_distance = dist_to_moon.y

        g = self._apply_gravity()

        gravitational_force = g.y

        a_gravity = gravitational_force / self.current_mass

        required_deceleration = v**2 / (2 * remaining_distance)

        # Calculate the actual thrust required to get the required deceleration
        thrust_needed = (required_deceleration + a_gravity) * self.current_mass

        # Adjust thrust dynamically, but don't exceed MAX_THRUST
        actual_thrust = min(thrust_needed, MAX_THRUST)
        
        return actual_thrust


    def compute_thrust(self):
        global at_moon
        pos, _ = p.getBasePositionAndOrientation(self.rocket_body)

        dist_to_moon = Vec3(moon_distance - moon_radius - pos[1] + rocket_height / 2)

        if dist_to_moon.y < r_initial:
            effective_thrust = -self.compute_thrust_v2()
            self.landing = True
        else:
            effective_thrust = self.thrust_force * self.num_engines

        if dist_to_moon.y <= 42:
            self.landing = False
            effective_thrust = 0
            at_moon = True

        thrust_force_vector = Vec3(0, effective_thrust, 0)

        return thrust_force_vector

    def _consume_fuel_and_apply_thrust(self):
        global held_keys, oxygen_level, total_rocket_mass


        thrust_force_vector = self.compute_thrust()

        #print(LAGRANGE_DISTANCE - PROXIMITY_THRESHOLD)
        global start_time

        if ((self.current_fuel_mass > 0) and held_keys['space']) or self.landing or held_keys['t']:
        #if held_keys['space']:
            rocket_position, _ = p.getBasePositionAndOrientation(self.rocket_body)

            if self.landing or held_keys['t']:

                #effective_thrust = self.thrust_force * self.num_engines

                force_application_point = [0, rocket_height / 2, 0]
                p.applyExternalForce(self.rocket_body, -1, thrust_force_vector, force_application_point, p.LINK_FRAME)
            else:
                force_application_point = [0, -rocket_height / 2, 0]
                p.applyExternalForce(self.rocket_body, -1, thrust_force_vector, force_application_point, p.LINK_FRAME)

            # Apply thrust force to the rocket's physics body
            g = self._apply_gravity()
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
        # Calculate drag force
        velocity = p.getBaseVelocity(self.rocket_body)[0]
        velocity_magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        drag_force = 0.5 * air_density * velocity_magnitude**2 * drag_coefficient * frontal_area

        # Apply drag force in the opposite direction of velocity
        drag_force_direction = (-velocity[0] / velocity_magnitude, -velocity[1] / velocity_magnitude, -velocity[2] / velocity_magnitude) if velocity_magnitude > 0 else (0, 0, 0)
        drag_force_vec = (drag_force * drag_force_direction[0], drag_force * drag_force_direction[1], drag_force * drag_force_direction[2])

        rocket_position, _ = p.getBasePositionAndOrientation(self.rocket_body)

        #p.applyExternalForce(self.rocket_body, -1, drag_force_vec, rocket_position, p.LINK_FRAME)


    def _update_info(self):
        rocket_pos, _ = p.getBasePositionAndOrientation(self.rocket_body)
        self.altitude = (rocket_pos[1] - earth_radius)

    def _update_position(self):
        pos, rot = p.getBasePositionAndOrientation(self.rocket_body)
        self.entity.position = Vec3(0, pos[1] * scale_radius, 0)
        rocket.entity.rotation =  euler_to_degrees(rot)


class Moon:
    def __init__(self):
        self.entity = Entity(model='sphere', color=color.gray, scale=moon_radius_game)
        self.entity.world_position = earth.entity.world_position + Vec3(0, moon_distance_game, 0)

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
            self.entity.clip_plane_near = 0.01
            self.entity.clip_plane_far = earth_radius * 1000
            self.entity.fov = 10

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
    
    # Create the rocket
    rocket_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[rocket_radius, rocket_height / 2, rocket_radius])
    rocket_body = p.createMultiBody(baseMass=total_rocket_mass, baseCollisionShapeIndex=rocket_shape, basePosition=rocket_start_position)

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

    rocket.entity.position = Vec3(0, pos[1] * scale_radius, 0)
    rocket.entity.rotation =  euler_to_degrees(new_euler)


    #rocket.entity.rotation = [0, 90, 90]



def super_update(loop_time):
    for x in range (0, loop_time):
        p.stepSimulation()
        
        update_minimap()

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

at_moon = False
# Update function
def update():
    global at_moon
    pos, _ = p.getBasePositionAndOrientation(rocket.rocket_body)

    if at_moon:
        #moon_landing()

        return

    super_update(64)
    


app.run()
