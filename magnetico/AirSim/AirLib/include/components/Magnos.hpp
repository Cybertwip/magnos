#pragma once

#include "components/systems/MagnetSystem.hpp"
#include "components/IronBall.hpp"
#include "components/entities/CoilEntity.hpp"
#include "components/systems/AlternatorSystem.hpp"
#include "components/systems/CoilSystem.hpp"

class Magnos : public msr::airlib::Node {
private:
	
	static constexpr float MU_0 = 4.0f * M_PI * 1e-7f;  // Vacuum permeability
	static constexpr float B0 = 0.01;  // Example value in teslas for a standard lab magnet

public:	
	float calculateFlux(CoilEntity::AttachedEntity& coil, MagnetEntity::AttachedEntity& magnet);
	
	float calculateCoilEMF(const CoilEntity::AttachedEntity& coil, float delta);
	
	void update(float);
	
	
	float calculateEffectiveArea(const msr::airlib::Quaternionr& rotation);
	
	void applyTorqueAndRotate(std::shared_ptr<Node> node, const msr::airlib::Vector3r& torque, float delta, const msr::airlib::Vector3r& axis);
	void applyMagneticImpulse(float delta);
	
	AlternatorSystem& getAlternatorSystem();
	CoilSystem& getCoilSystem();
	
	std::shared_ptr<Node> outerNode;
	std::shared_ptr<Node> middleNode;
	std::shared_ptr<Node> innerNode;
	
	std::shared_ptr<MagneticBall> pinball;
	
	std::unique_ptr<CoilSystem> outerCoilSystem;
	
	AlternatorSystem alternator = AlternatorSystem(0.04f, 40);
	
	std::unique_ptr<MagnetSystem> middleMagnetSystem = std::make_unique<MagnetSystem>();
	std::unique_ptr<MagnetSystem> outerMagnetSystem = std::make_unique<MagnetSystem>();

	std::unique_ptr<MagnetSystem> innerMagnetSystem = std::make_unique<MagnetSystem>();

	float baseDistanceOffset = 0.07f; // Base offset for the distance
	
	float innerRingRadius = 0.05f; // Base offset for the distance
	float middleRingRadius = 0.06f; // Base offset for the distance
	float outerRingRadius = 0.10f; // Base offset for the distance

public:
	void init();
	void loadData(int id);
	
	void attachPinball();
};
