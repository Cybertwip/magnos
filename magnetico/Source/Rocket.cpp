#include "Rocket.h"
#include "Settings.h"

#include <cmath>

Rocket::Rocket(float mass,
			   float thrust,
			   float first_stage_mass,
			   float second_stage_mass,
			   float first_stage_thrust,
			   float second_stage_thrust,
			   float specific_impulse,
			   float burn_rate,
			   float nozzle_radius,
			   float drag_coefficient,
			   float cross_sectional_area,
			   float initial_propellant_mass,
			   float valve_multiplier,
			   float max_valve_opening,
			   float max_thrust_multiplier,
			   float max_specific_impulse_multiplier,
			   float max_burn_rate_multiplier,
			   float max_nozzle_radius_multiplier,
			   float max_drag_coefficient_multiplier,
			   float max_cross_sectional_area_multiplier,
			   float max_propellant_mass_multiplier,
			   float max_mass_multiplier,
			   ax::Vec3 initial_position,
			   ax::Quaternion initial_orientation) :
dry_mass(mass - initial_propellant_mass - first_stage_mass - second_stage_mass),
thrust_kN(thrust),
first_stage_thrust(first_stage_thrust),
second_stage_thrust(second_stage_thrust),
specific_impulse(specific_impulse),
burn_rate(burn_rate),
nozzle_radius(nozzle_radius),
drag_coefficient(drag_coefficient),
cross_sectional_area(cross_sectional_area),
initial_propellant_mass(initial_propellant_mass),
valve_multiplier(valve_multiplier),
max_valve_opening(max_valve_opening),
max_thrust_multiplier(max_thrust_multiplier),
max_specific_impulse_multiplier(max_specific_impulse_multiplier),
max_burn_rate_multiplier(max_burn_rate_multiplier),
max_nozzle_radius_multiplier(max_nozzle_radius_multiplier),
max_drag_coefficient_multiplier(max_drag_coefficient_multiplier),
max_cross_sectional_area_multiplier(max_cross_sectional_area_multiplier),
max_propellant_mass_multiplier(max_propellant_mass_multiplier),
max_mass_multiplier(max_mass_multiplier),
current_stage(0),
initial_position(initial_position),
initial_orientation(initial_orientation)
{
	// Create the rocket collision shape
	CollisionShape* rocket_shape = new CylinderShape(rocket_radius_game, rocket_height_game / 2);
	
	// Calculate the initial mass of the rocket
	current_mass = dry_mass + initial_propellant_mass;
	
	// Calculate the initial thrust of the rocket
	current_thrust = thrust_kN * thrust_multiplier * valve_multiplier;
	
	// Calculate the initial specific impulse of the rocket
	current_specific_impulse = isp_multiplier * 9.81 * specific_impulse;
	
	// Calculate the initial burn rate of the rocket
	current_burn_rate = burn_rate * valve_multiplier;
	
	// Calculate the initial nozzle radius of the rocket
	current_nozzle_radius = nozzle_radius * max_nozzle_radius_multiplier * valve_multiplier;
	
	// Calculate the initial cross-sectional area of the rocket
	current_cross_sectional_area = cross_sectional_area * max_cross_sectional_area_multiplier * valve_multiplier;
	
	// Calculate the initial drag coefficient of the rocket
	current_drag_coefficient = drag_coefficient * max_drag_coefficient_multiplier;
	
	// Calculate the initial propellant mass of the rocket
	current_propellant_mass = initial_propellant_mass;
		
	// Create the rocket rigid body
	ax::Vec3 rocket_inertia(0, 0, 0);
	rocket_shape->calculateLocalInertia(current_mass, rocket_inertia);
	
	RigidBodyConstructionInfo rocket_rb_info(current_mass, {}, {}, {}, 0, 0);
	rocket_body = rocket_rb_info.createRigidBody();
}


Rocket::~Rocket() {
	delete rocket_body;
}

void Rocket::update(float dt) {
	// Calculate the current mass of the rocket
	if (current_stage == 1) {
		current_mass = first_stage_mass + current_propellant_mass;
	} else {
		current_mass = second_stage_mass + current_propellant_mass;
	}
	_update_mass();
	
	// Calculate the current thrust of the rocket
	if (current_stage == 1) {
		current_thrust = first_stage_thrust * thrust_multiplier * valve_multiplier;
	} else {
		current_thrust = second_stage_thrust * thrust_multiplier * valve_multiplier;
	}
	
	// Calculate the current specific impulse of the rocket
	current_specific_impulse = isp_multiplier * 9.81 * specific_impulse;
	
	// Calculate the current burn rate of the rocket
	current_burn_rate = burn_rate * valve_multiplier;
	
	// Calculate the current nozzle radius of the rocket
	current_nozzle_radius = nozzle_radius * max_nozzle_radius_multiplier * valve_multiplier;
	
	// Calculate the current cross-sectional area of the rocket
	current_cross_sectional_area = cross_sectional_area * max_cross_sectional_area_multiplier * valve_multiplier;
	
	// Calculate the current drag coefficient of the rocket
	current_drag_coefficient = drag_coefficient * max_drag_coefficient_multiplier;
	
	// Calculate the current propellant mass of the rocket
	float propellant_mass_delta = -current_burn_rate * dt;
	current_propellant_mass += propellant_mass_delta;
	current_propellant_mass = std::max(0.0f, current_propellant_mass);  // Limit to a minimum of 0
	
	// Calculate the current mass of the rocket
	if (current_stage == 1) {
		current_mass = first_stage_mass + current_propellant_mass;
	} else {
		current_mass = second_stage_mass + current_propellant_mass;
	}
	_update_mass();
	
	// Apply the current thrust to the rocket
	ax::Vec3 thrust_vector(0, current_thrust, 0);
	ax::Quaternion orientation_quaternion = get_rocket_orientation();
	ax::Vec3 orientation_euler = quaternion_to_euler(orientation_quaternion);
	ax::Mat4 orientation_matrix;
	ax::Mat4::createRotation(orientation_quaternion, &orientation_matrix);
	ax::Vec3 thrust_direction = orientation_matrix * ax::Vec3(0, 1, 0);
	ax::Vec3 thrust_force = thrust_direction * current_thrust;
	rocket_body->applyForce(thrust_force);
	
	// Apply atmospheric drag to the rocket
	ax::Vec3 rocket_velocity = rocket_body->getLinearVelocity();
	float rocket_speed = rocket_velocity.length();
	ax::Vec3 rocket_direction = rocket_velocity.getNormalized();
	ax::Vec3 rocket_position = rocket_body->getCenterOfMassPosition();
	float altitude = rocket_position.length() - earth_radius_game;
	float atmospheric_density = _get_atmospheric_density(altitude);
	ax::Vec3 drag_force = -0.5 * atmospheric_density * current_drag_coefficient * current_cross_sectional_area * rocket_speed * rocket_speed * rocket_direction;
	rocket_body->applyForce(drag_force);
}

float Rocket::get_first_stage_mass() {
	return first_stage_mass;
}

float Rocket::get_second_stage_mass() {
	return second_stage_mass;
}

float Rocket::get_mass() {
    float mass = rocket_body->getInvMass();
    if (mass == 0) {
        return 0;
    }
    return 1 / mass;
}

float Rocket::_get_rocket_radius() {
	return rocket_radius_game;
}

ax::Vec3 Rocket::get_position() {
	return rocket_body->getCenterOfMassPosition();
}

ax::Quaternion Rocket::get_rocket_orientation() {
//    btTransform transform;
//	rocket_body->getMotionState()->getWorldTransform(transform);
//    return transform.getRotation();
	
	return ax::Quaternion::identity();
}

ax::Vec3 Rocket::quaternion_to_euler(ax::Quaternion quaternion) {
	ax::Vec3 euler_angles;
	// Convert the quaternion to a rotation matrix
	ax::Mat4 rotation_matrix;
	ax::Mat4::createRotation(quaternion, &rotation_matrix);
	// Extract the Euler angles from the rotation matrix
	euler_angles.x = atan2(rotation_matrix.m[8], rotation_matrix.m[10]);
	euler_angles.y = atan2(-rotation_matrix.m[9], sqrt(pow(rotation_matrix.m[8], 2) + pow(rotation_matrix.m[10], 2)));
	euler_angles.z = atan2(rotation_matrix.m[1], rotation_matrix.m[0]);
	return euler_angles;
}

float Rocket::_get_atmospheric_pressure(float altitude) {
	float pressure = 101325.0f * pow((1 - (0.0065f * altitude) / 288.15f), 5.255f);
	return pressure;
}

float Rocket::_get_atmospheric_density(float altitude) {
    const float R = 287.058f; // Specific gas constant for dry air
    const float g0 = 9.80665f; // Standard gravitational acceleration at sea level
    const float T0 = 288.15f; // Standard temperature at sea level
    const float p0 = 101325.0f; // Standard pressure at sea level
    const float L = 0.0065f; // Temperature lapse rate
    const float M = 0.0289644f; // Molar mass of dry air

    float T = T0 - L * altitude;
    float p = p0 * powf(1 - L * altitude / T0, g0 * M / (R * L));
    float rho = p * M / (R * T);

    return rho;
}

float Rocket::_get_gravitational_force(float altitude) {
    const float G = 6.6743e-11f; // Gravitational constant
    const float M = 5.9722e24f; // Mass of the Earth
    const float R = 6371000.0f; // Radius of the Earth

    float h = altitude + R;
    float Fg = G * M * current_mass / (h * h);

    return Fg;
}

float Rocket::_get_drag_force(float altitude, float velocity) {
    float rho = _get_atmospheric_density(altitude);
    float v = velocity;
    float Cd = _get_drag_coefficient();
    float A = _get_cross_sectional_area();

    float Fd = 0.5f * rho * v * v * Cd * A;

    return Fd;
}

float Rocket::_get_thrust() {
	float thrust = current_thrust * valve_multiplier * max_thrust_multiplier;
	return thrust;
}

float Rocket::_get_thrust_force(float altitude) {
    float T = _get_thrust();
    float Pe = _get_nozzle_radius();
    float Pa = _get_atmospheric_pressure(altitude);
    float rho = _get_atmospheric_density(altitude);
    float Isp = _get_specific_impulse();
    float mdot = T / (Isp * 9.81f);
    float Ae = M_PI * Pe * Pe;
    float Ar = M_PI * _get_rocket_radius() * _get_rocket_radius();
    float er = Ae / Ar;
    float F = mdot * Isp * 9.81f * er * (1 - (Pa / rho));

    return F;
}

float Rocket::_get_specific_impulse() {
    return current_specific_impulse;
}

float Rocket::_get_burn_rate() {
    return current_burn_rate;
}

float Rocket::_get_nozzle_radius() {
    return current_nozzle_radius;
}

float Rocket::_get_drag_coefficient() {
    return current_drag_coefficient;
}

float Rocket::_get_cross_sectional_area() {
    return current_cross_sectional_area;
}

float Rocket::_get_propellant_mass() {
    return current_propellant_mass;
}

float Rocket::_get_valve_opening() {
    return current_valve_opening;
}

void Rocket::_update_mass() {
    current_mass = dry_mass + first_stage_mass + second_stage_mass + current_propellant_mass;
}

void Rocket::_update_thrust() {
    current_thrust = _get_thrust();
}

void Rocket::_update_specific_impulse() {
    current_specific_impulse = specific_impulse;
}

void Rocket::_update_burn_rate() {
    current_burn_rate = burn_rate * current_valve_opening * valve_multiplier;
}

void Rocket::_update_nozzle_radius() {
    current_nozzle_radius = nozzle_radius;
}

void Rocket::_update_drag_coefficient() {
    current_drag_coefficient = drag_coefficient;
}

void Rocket::_update_cross_sectional_area() {
    current_cross_sectional_area = cross_sectional_area;
}

void Rocket::_update_propellant_mass(float delta_time) {
    current_propellant_mass -= current_burn_rate * delta_time;
    if (current_propellant_mass < 0) {
        current_propellant_mass = 0;
    }
}

void Rocket::_update_valve_opening() {
    if (held_keys.empty()) {
        valve_multiplier = 0.0f;
    } else {
        valve_multiplier = 1.0f;
    }
    if (std::find(held_keys.begin(), held_keys.end(), 'w') != held_keys.end()) {
        current_valve_opening += 0.1f;
    }
    if (std::find(held_keys.begin(), held_keys.end(), 's') != held_keys.end()) {
        current_valve_opening -= 0.1f;
    }
    if (current_valve_opening < 0) {
        current_valve_opening = 0;
    }
    if (current_valve_opening > 1) {
        current_valve_opening = 1;
    }
}

void Rocket::handle_input(char key, bool is_pressed) {
    if (is_pressed) {
        held_keys.push_back(key);
    } else {
        held_keys.erase(std::remove(held_keys.begin(), held_keys.end(), key), held_keys.end());
    }
}

void Rocket::adjust_valve_opening() {
    if (held_keys.empty()) {
        valve_multiplier = 0.0f;
    } else {
        valve_multiplier = 1.0f;
    }
    if (std::find(held_keys.begin(), held_keys.end(), 'w') != held_keys.end()) {
        current_valve_opening += 0.1f;
    }
    if (std::find(held_keys.begin(), held_keys.end(), 's') != held_keys.end()) {
        current_valve_opening -= 0.1f;
    }
    if (current_valve_opening < 0) {
        current_valve_opening = 0;
    }
    if (current_valve_opening > 1) {
        current_valve_opening = 1;
    }
}

void Rocket::jettison_first_stage() {
    if (first_stage_mass > 0) {
        rocket_mass -= first_stage_mass;
        first_stage_mass = 0;
    }
}

void Rocket::jettison_second_stage() {
    if (second_stage_mass > 0) {
        rocket_mass -= second_stage_mass;
        second_stage_mass = 0;
    }
}

void Rocket::set_gimbal_rotation(float angle) {
//    if (angle > max_gimbal_angle) {
//        angle = max_gimbal_angle;
//    }
//    if (angle < -max_gimbal_angle) {
//        angle = -max_gimbal_angle;
//    }
//    current_gimbal_angle = angle;
}

void Rocket::set_thrust(float thrust) {
    this->thrust = thrust;
}

void Rocket::set_specific_impulse(float isp) {
    specific_impulse = isp;
}

void Rocket::set_burn_rate(float burn_rate) {
    this->burn_rate = burn_rate;
}

void Rocket::set_nozzle_radius(float radius) {
    nozzle_radius = radius;
}

void Rocket::set_drag_coefficient(float drag_coefficient) {
    this->drag_coefficient = drag_coefficient;
}

void Rocket::set_cross_sectional_area(float cross_sectional_area) {
    this->cross_sectional_area = cross_sectional_area;
}

void Rocket::set_propellant_mass(float mass) {
    current_propellant_mass = mass;
}

void Rocket::set_valve_opening(float opening) {
    current_valve_opening = opening;
}
