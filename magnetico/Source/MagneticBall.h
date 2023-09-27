#pragma once

#include <axmol.h>

class MagneticBall : public ax::MeshRenderer {
protected:
	float radius;
	virtual float get_density() const = 0;
	float calculate_mass() const;
	float calculate_permeability() const;
		
	ax::Mesh* createSphere(float radius, unsigned int rings, unsigned int sectors);

public:
	MagneticBall(float r);
	
	virtual ~MagneticBall();
	
	virtual float get_magnetization() const;

	float calculate_inertia() const;

	float getForceDueToMagneticField(const ax::Vec3& B, const ax::Vec3& gradB) const;

	float calculate_volume() const;

	virtual float get_relative_permeability() const = 0;
};
