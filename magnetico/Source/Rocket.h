#pragma once 

#include "Physics.h"

#include <axmol.h>

#include <vector>


class Rocket : public ax::Node {
public:
	Rocket(float mass,
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
		   float max_valve_opening,
		   ax::Vec3 initial_position,
		   ax::Quaternion initial_orientation);
	
public:
	~Rocket();
	void update(float dt);
	float get_mass();
	float get_first_stage_mass();
	float get_second_stage_mass();
	ax::Vec3 get_position();
	ax::Quaternion get_rocket_orientation();
	ax::Vec3 quaternion_to_euler(ax::Quaternion quaternion);
	
	
private:
	float _get_rocket_radius();
	float _get_atmospheric_pressure(float altitude);
	float _get_atmospheric_density(float altitude);
	float _get_gravitational_force(float altitude);
	float _get_drag_force(float altitude, float velocity);
	float _get_thrust();
	float _get_thrust_force(float altitude);
	float _get_specific_impulse();
	float _get_burn_rate();
	float _get_nozzle_radius();
	float _get_drag_coefficient();
	float _get_cross_sectional_area();
	float _get_propellant_mass();
	float _get_valve_opening();
	void _update_mass();
	void _update_thrust();
	void _update_specific_impulse();
	void _update_burn_rate();
	void _update_nozzle_radius();
	void _update_drag_coefficient();
	void _update_cross_sectional_area();
	void _update_propellant_mass(float deltaTime);
	void _update_valve_opening();
	void handle_input(char key, bool is_pressed);
	void adjust_valve_opening();
	void jettison_first_stage();
	void jettison_second_stage();
	void set_gimbal_rotation(float angle);
	void set_thrust(float thrust);
	void set_specific_impulse(float isp);
	void set_burn_rate(float burn_rate);
	void set_nozzle_radius(float radius);
	void set_drag_coefficient(float drag_coefficient);
	void set_cross_sectional_area(float cross_sectional_area);
	void set_propellant_mass(float mass);
	void set_valve_opening(float opening);
	
private:
	float thrust;
	float first_stage_thrust;
	float second_stage_thrust;
	float rocket_mass;

private:
	float current_mass;
	float current_thrust;
	float current_specific_impulse;
	float current_burn_rate;
	float current_nozzle_radius;
	float current_drag_coefficient;
	float current_cross_sectional_area;
	float current_propellant_mass;
	float max_valve_opening;
	float current_valve_opening;
	
	int current_stage;
	ax::Vec3 initial_position;
	ax::Quaternion initial_orientation;
	RigidBody* rocket_body;
	
	const float rocket_radius_game = 0.5f;
	const float rocket_height_game = 2.0f;
	
private:
	float dry_mass;
	float thrust_kN;
	float specific_impulse;
	float burn_rate;
	float nozzle_radius;
	float drag_coefficient;
	float cross_sectional_area;
	float initial_propellant_mass;
	
	const float earth_radius_game = 6.371e6f;
	const float earth_mass = 5.972e24f;
	
	float first_stage_mass;
	float second_stage_mass;
	
	std::vector<char> held_keys;

};
