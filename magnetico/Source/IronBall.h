#pragma once

#include "MagneticBall.h"

class IronBall : public MagneticBall {
	static constexpr float density = 7870.0f;  // kg/m^3
	static constexpr float relative_permeability = 5000.0f;  // This is a general value; actual value can vary widely
	static constexpr float M_s = 1.7e6;  // Saturation magnetization for Iron (in A/m)

private:
	IronBall(float radius);

protected:
	float get_density() const override;

	
	float get_relative_permeability() const;
	
	float get_magnetization() const;

public:
	static IronBall* create(float radius);
};