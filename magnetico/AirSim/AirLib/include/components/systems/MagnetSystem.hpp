#pragma once

#include "components/entities/MagnetEntity.hpp"
#include "components/Magnet.hpp"

class MagnetSystem : public MagnetEntity {
private:
	static constexpr float mu_0 = 4 * M_PI * 1e-7;  // Vacuum permeability
	//float _magneticFieldStrength = 0.0015f;  // T, approximate value for N52 neodymium bar magnet
	float _magneticFieldStrength = 0.03f;  // T, approximate value for a common ferrite magnet

	std::vector<std::shared_ptr<Magnet>> magnets;

public:

	MagnetSystem();
	~MagnetSystem() override = default;
	
	msr::airlib::Vector3r calculateMagneticFieldAtOrigin(const msr::airlib::Vector3r& origin, const msr::airlib::Vector3r& magnetPosition, MagnetPolarity polarity);
	
	msr::airlib::Vector3r calculateForceDueToMagnet(const msr::airlib::Vector3r& origin, const msr::airlib::Vector3r& magnetPosition, const msr::airlib::Vector3r& affectedMagnetPosition, MagnetPolarity polarity);
	
	msr::airlib::Vector3r combineFieldsOrForces(const msr::airlib::Vector3r&) override;
	
	std::shared_ptr<Node> attach(float radius, MagnetDirection direction, MagnetPolarity polarity);
	
	void update();

};
