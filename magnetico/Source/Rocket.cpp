#include "Rocket.h"
#include "Settings.h"
#include "Utils3d.h"

#include <cmath>

Rocket::Rocket(float mass,
			   float thrust,
			   float first_stage_mass,
			   float second_stage_mass,
			   float specific_impulse,
			   float burn_rate,
			   float nozzle_radius,
			   float drag_coefficient,
			   float cross_sectional_area,
			   float initial_propellant_mass,
			   float valve_multiplier,
			   ax::Vec3 initial_position,
			   ax::Quaternion initial_orientation) :
dry_mass(mass - initial_propellant_mass),
thrust_kN(thrust),
first_stage_mass(first_stage_mass),
second_stage_mass(second_stage_mass),
specific_impulse(specific_impulse),
burn_rate(burn_rate),
nozzle_radius(nozzle_radius),
drag_coefficient(drag_coefficient),
cross_sectional_area(cross_sectional_area),
initial_propellant_mass(initial_propellant_mass),
max_valve_opening(max_valve_opening),
initial_position(initial_position),
initial_orientation(initial_orientation)
{
	// Create the rocket collision shape
	CollisionShape* rocket_shape = new CylinderShape(rocket_radius, rocket_height / 2);
	
	// Calculate the initial mass of the rocket
	current_mass = dry_mass + initial_propellant_mass;
	
	// Calculate the initial thrust of the rocket
	current_thrust = thrust_kN;
	
	// Calculate the initial specific impulse of the rocket
	current_specific_impulse = 9.81 * specific_impulse;
	
	// Calculate the initial burn rate of the rocket
	current_burn_rate = burn_rate;
	
	// Calculate the initial nozzle radius of the rocket
	current_nozzle_radius = nozzle_radius;
	
	// Calculate the initial cross-sectional area of the rocket
	current_cross_sectional_area = cross_sectional_area;
	
	// Calculate the initial drag coefficient of the rocket
	current_drag_coefficient = drag_coefficient;
	
	// Calculate the initial propellant mass of the rocket
	current_propellant_mass = initial_propellant_mass;
		
	// Create the rocket rigid body
	ax::Vec3 rocket_inertia(0, 0, 0);
	rocket_shape->calculateLocalInertia(current_mass, rocket_inertia);
	
	RigidBodyConstructionInfo rocket_rb_info(current_mass, initial_position, {}, {}, 0, 0);
	rocket_body = rocket_rb_info.createRigidBody();

	autorelease();
	
	auto renderer = ax::MeshRenderer::create("soyuz-fg.obj");
	renderer->setPosition3D(ax::Vec3(0, 0, 0));
	renderer->setColor(ax::Color3B::GRAY);
	//	renderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::QUAD_TEXTURE, false));
//	renderer->setTexture("gold.jpg");
	this->addChild(renderer);
}


Rocket::~Rocket() {
	delete rocket_body;
}

void Rocket::update(float dt) {
	// Calculate the current mass of the rocket
	_update_mass();
	
	// Calculate the current thrust of the rocket
	_update_thrust();

	// Calculate the current specific impulse of the rocket
	current_specific_impulse = 9.81 * specific_impulse;
	
	// Calculate the current burn rate of the rocket
	current_burn_rate = burn_rate;
	
	// Calculate the current nozzle radius of the rocket
	current_nozzle_radius = nozzle_radius;
	
	// Calculate the current cross-sectional area of the rocket
	current_cross_sectional_area = cross_sectional_area;
	
	// Calculate the current drag coefficient of the rocket
	current_drag_coefficient = drag_coefficient;
		
	_update_propellant_mass(dt);
	
	// Apply the current thrust to the rocket
	ax::Vec3 rocket_position = rocket_body->getPosition();

	altitude = rocket_position.length() - earth_radius;

	float force = _get_thrust_force(altitude);
	
	ax::Vec3 thrust_vector(0, force, 0);
	ax::Quaternion orientation_quaternion = get_rocket_orientation();
	ax::Mat4 orientation_matrix;
	ax::Mat4::createRotation(orientation_quaternion, &orientation_matrix);
	ax::Vec3 thrust_direction = orientation_matrix * ax::Vec3(0, -1, 0);
	ax::Vec3 thrust_force = thrust_direction * force; // Corrected direction
	rocket_body->applyForce(thrust_force / 1000.0f);

	
	// Apply atmospheric drag to the rocket
	ax::Vec3 rocket_velocity = rocket_body->getLinearVelocity();
	float rocket_speed = rocket_velocity.length();
	ax::Vec3 rocket_direction = rocket_velocity.getNormalized();
	float atmospheric_density = _get_atmospheric_density(altitude);
	ax::Vec3 drag_force_components = -0.5 * atmospheric_density * current_drag_coefficient * current_cross_sectional_area * rocket_speed * rocket_speed * rocket_direction;
	
	// Now, calculate the total drag force as a vector
	ax::Vec3 drag_force = ax::Vec3(
								   drag_force_components.x * rocket_velocity.x,
								   drag_force_components.y * rocket_velocity.y,
								   drag_force_components.z * rocket_velocity.z
								   );
	rocket_body->applyForce(-drag_force / 1000.0f);
	rocket_body->update(dt);
	this->setPosition3D(rocket_body->getPosition());
}

float Rocket::get_first_stage_mass() {
	return first_stage_mass;
}

float Rocket::get_second_stage_mass() {
	return second_stage_mass;
}

float Rocket::get_mass() {
    float mass = current_mass;
    if (mass == 0) {
        return 0;
    }
    return current_mass;
}

float Rocket::_get_rocket_radius() {
	return rocket_radius;
}

ax::Vec3 Rocket::get_position() {
	return rocket_body->getCenterOfMassPosition();
}

float Rocket::get_altitude() {
	return altitude;
}

float Rocket::get_linear_velocity() {
	ax::Vec3 rocket_velocity = rocket_body->getLinearVelocity();
	return rocket_velocity.length();
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
	float denominator = 1 - L * altitude / T0;
	if(denominator == 0){
		return 0;
	}
	float p = p0 * powf(denominator, g0 * M / (R * L));
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
	float thrust = current_thrust;
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
	float specific_impulse_at_sea_level = 311.0f; // seconds
	float specific_impulse_in_vacuum = 282.0f; // seconds
	float sea_level_pressure = 101325.0f; // Pascals
	float vacuum_pressure = 0.0f; // Pascals
	
	float pressure = sea_level_pressure * exp(-altitude / 8000.0f); // simple atmospheric model
	float specific_impulse = specific_impulse_at_sea_level + (specific_impulse_in_vacuum - specific_impulse_at_sea_level) * (pressure - vacuum_pressure) / (sea_level_pressure - vacuum_pressure);
	return specific_impulse;
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
    current_mass = dry_mass + current_propellant_mass;
}

void Rocket::_update_thrust() {
    current_thrust = _get_thrust();
}

void Rocket::_update_specific_impulse() {
    current_specific_impulse = specific_impulse;
}

void Rocket::_update_burn_rate() {
    current_burn_rate = burn_rate * current_valve_opening;
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
