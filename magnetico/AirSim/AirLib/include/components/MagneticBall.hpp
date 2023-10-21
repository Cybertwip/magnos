#pragma once

#include "components/entities/Node.hpp"

class MagneticBall : public msr::airlib::Node {
protected:
	float radius;
	virtual float get_density() const = 0;
	float calculate_mass() const;
	float calculate_permeability() const;
		
public:
	MagneticBall(float r);
	
	virtual ~MagneticBall();
	
	virtual float get_magnetization() const;

	float calculate_inertia() const;

	float getForceDueToMagneticField(const msr::airlib::Vector3r& B, const msr::airlib::Vector3r& gradB) const;

	float calculate_volume() const;

	virtual float get_relative_permeability() const = 0;
};
