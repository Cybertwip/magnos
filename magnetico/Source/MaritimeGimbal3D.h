#pragma once

#include "CustomNode.h"
#include "MagnetSystem.h"
#include "IronBall.h"
#include "CoilEntity.h"
#include "AlternatorSystem.h"
#include "CoilSystem.h"

#include <axmol.h>

#include <vector>

class MaritimeGimbal3D : public ax::Node {
private:
	const float EARTH_MAGNETIC_FIELD_STRENGTH = 50e-6; // 50 microteslas as an average value
	const float SIMULATION_SCALE = 1; // Arbitrary scale to make the force tangible in the simulation

	
	static constexpr float MU_0 = 4.0f * M_PI * 1e-7f;  // Vacuum permeability
	static constexpr float B0 = 0.01;  // Example value in teslas for a standard lab magnet

	ax::Mesh* createPole(float height, float radius, int segments);
public:
	void addRodsToIronBall(IronBall* ball, float rodLength, float rodRadius);
	
	// Helper function to convert degrees to radians
	float degToRad(float degrees);
	
	// Helper function to convert radians to degrees
	float radToDeg(float radians);

	// This function returns a direction vector given an angle from the North (positive Z-axis)
	ax::Vec3 directionFromAngle(float angleInDegrees);
	
	void addPoles(ax::Node* node, float innerRadius, float distance, float poleRadius, const ax::Vec3& direction);
	void addPoles(ax::Node* node, float innerRadius, float distance, float poleRadius, float angleFromNorth);
	ax::Mesh* createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments);
	ax::Mesh* createFlatDisk(float radius, float thickness, int majorSegments, int minorSegments);
	
	ax::Vec3 rotateAroundAxis(const ax::Vec3& point, const ax::Vec3& axis, float angle);
	
	ax::Vec3 calculateMagneticFieldAt(const ax::Vec3& position);
	
	float calculateFluxThroughCoil(const ax::Vec3& B, float coilArea, int coilTurns);
	
	void calculateFlux(CoilEntity::AttachedEntity& coil, const ax::Vec3& position);
	
	float calculateCoilEMF(const CoilEntity::AttachedEntity& coil, float delta);
	
	void update(float) override;
	
	
	float calculateEffectiveArea(const ax::Quaternion& rotation);
	
	void applyTorqueAndRotate(CustomNode* node, const ax::Vec3& torque, float delta, ax::Vec3 axis);
	void applyMagneticImpulse(float delta);
	
	AlternatorSystem& getAlternatorSystem();
	CoilSystem& getCoilSystem();
	
	CustomNode* outerNode;
	CustomNode* middleNode;
	CustomNode* innerNode;
	
	ax::Mesh* outerRing;
	ax::Mesh* middleRing;
	ax::Mesh* innerRing;
	ax::Node* pinball;
	
	std::unique_ptr<CoilSystem> outerCoilSystem =
	std::make_unique<CoilSystem>(1.5f,
								 1.0f, // resistance
								 0.5f, // current
								 360); // turns
	
	AlternatorSystem alternator = AlternatorSystem(0.025f, 420);
	
	std::unique_ptr<MagnetSystem> middleMagnetSystem = std::make_unique<MagnetSystem>();
	
	std::unique_ptr<MagnetSystem> innerMagnetSystem = std::make_unique<MagnetSystem>();

	float baseDistanceOffset = 0.07f; // Base offset for the distance
	
	float innerRingRadius = 0.05f; // Base offset for the distance
	float middleRingRadius = 0.06f; // Base offset for the distance
	float outerRingRadius = 0.12f; // Base offset for the distance

public:
	CREATE_FUNC(MaritimeGimbal3D);
	
	bool init() override;
	
	void attachPinball(ax::Scene* scene);
};
