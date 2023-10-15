#include "components/Magnos.hpp"

namespace {
// Function to create a quaternion from an axis-angle representation
msr::airlib::Quaternionr createQuaternionFromAxisAngle(const msr::airlib::Vector3r& axis, float rotationAngle) {
	// Calculate the sine and cosine of half the angle
	float halfAngle = rotationAngle / 2.0f;
	float sinHalfAngle = sin(halfAngle);
	float cosHalfAngle = cos(halfAngle);
	
	// Return the quaternion
	return msr::airlib::Quaternionr(cosHalfAngle, axis.x() * sinHalfAngle, axis.y() * sinHalfAngle, axis.z() * sinHalfAngle);
}


// Helper function to convert degrees to radians
float degToRad(float degrees) {
	return degrees * (M_PI / 180.0f);
}

// Helper function to convert radians to degrees
float radToDeg(float radians) {
	return radians * (180.0f / M_PI);
}

msr::airlib::Vector3r directionFromAngle(float angleInDegrees) {
	float rad = degToRad(angleInDegrees);
	return msr::airlib::Vector3r(cos(rad), sin(rad), 0);
}


}


// This function returns a direction vector given an angle from the North (positive Z-axis)
void Magnos::calculateFlux(CoilEntity::AttachedEntity& coil, MagnetEntity::AttachedEntity& magnet) {
	auto ironBall = std::dynamic_pointer_cast<MagneticBall>(pinball);
	float ballMagneticMoment = ironBall->get_magnetization() * ironBall->calculate_volume();
	
	msr::airlib::Vector3r r = magnet.position - coil.position;

	float distance = sqrt(pow(magnet.position.x() - coil.position.x(), 2) + pow(magnet.position.y() - coil.position.y(), 2) + pow(magnet.position.z() - coil.position.z(), 2));

	r.normalize();
	
	// Determine the magnetic field polarity based on both coil.polarity and magnet.polarity
	float coilPolarityFactor = (coil.polarity == MagnetPolarity::SOUTH) ? -1.0f : 1.0f;
	float magnetPolarityFactor = (magnet.polarity == MagnetPolarity::SOUTH) ? -1.0f : 1.0f;
	
	// Combine the polarity factors to determine the overall polarity effect
	float polarityFactor = coilPolarityFactor * magnetPolarityFactor;

	// Magnetic field at the coil's position due to the ball
	msr::airlib::Vector3r B = (MU_0 / (4.0f * M_PI)) * (polarityFactor * ballMagneticMoment / std::pow(distance, 3)) * r;

	float length = sqrt(B.x() * B.x() + B.y() * B.y() + B.z() * B.z());

	float flux = length * coil.area; // assuming coil's face is perpendicular to B
	
	// Update the flux through the coil due to this magnetic field
	coil.flux = flux;
}


float Magnos::calculateCoilEMF(const CoilEntity::AttachedEntity& coil, float delta) {
	// Change in flux
	float deltaPhi = coil.flux - coil.previousFlux;
	
	// EMF for this coil using Faraday's law
	return -coil.turns * (deltaPhi / delta);
}

void Magnos::update(float) {
	int cycles_per_collection = 1;
	
	if(getCoilSystem().collecting()){
		cycles_per_collection = Settings::data_collection_mode_cycles;
	}
	
	//global_delta = dt;
	
	//desired_voltage_per_second = desired_voltage * dt;
	int updates = 0;
	
	do{
		
		outerMagnetSystem->update();
		middleMagnetSystem->update();
		innerMagnetSystem->update();
		outerCoilSystem->update();
		alternator.update();
		
		auto ironBall = std::dynamic_pointer_cast<MagneticBall>(pinball);
		
		applyMagneticImpulse(Settings::fixed_delta);
		
		// Compute flux for all coils
		float totalEMF = 0.0f;
		auto& outerEntities = middleMagnetSystem->getAttachedEntities();
		auto& innerEntities = innerMagnetSystem->getAttachedEntities();
		
		auto& coils = alternator.getAttachedEntities();
		
		for (auto& coil : coils) {
			// Reset the coil's flux before summing from all entities
			bool firstRun = false;
			if(coil.flux == 0){
				firstRun = true;
			}
			coil.previousFlux = coil.flux;
			coil.flux = 0.0f;
			
			for (auto& entity : outerEntities) {
				// Compute flux for the current coil based on each entity's position
				calculateFlux(coil, entity);
			}
			
			if(firstRun){
				coil.previousFlux = coil.flux;
			}
			
			totalEMF += calculateCoilEMF(coil, Settings::fixed_delta);

		}
		

		// Compute total induced EMF in the alternator
		alternator.emf = totalEMF;
		
		outerCoilSystem->update(alternator.emf, Settings::global_delta);
		
		updates++;
	} while(updates < cycles_per_collection);
	
}
float Magnos::calculateEffectiveArea(const msr::airlib::Quaternionr& rotation) {
	// Original dimensions of the innerNode
	const float length = 1.0f;  // Replace with actual value
	const float width = 1.0f;   // Replace with actual value
	float A_original = length * width;
	
	// Assuming the normal vector of the unrotated innerNode is (0, 0, 1)
	msr::airlib::Vector3r unrotatedNormal(0, 0, 1);
	msr::airlib::Vector3r rotatedNormal = rotation * unrotatedNormal;  // Apply quaternion rotation to the normal
	
	// Assuming the magnetic field direction is vertical (0, 0, 1)
	msr::airlib::Vector3r magneticFieldDirection(0, 0, 1);
	
	// Compute the cosine of the angle between the rotated normal and the magnetic field direction
	float cosTheta = rotatedNormal.dot(magneticFieldDirection);
	
	return A_original * cosTheta;
}

void Magnos::applyTorqueAndRotate(std::shared_ptr<Node> node, const msr::airlib::Vector3r& torque, float delta, const msr::airlib::Vector3r& axis) {
	auto magneticBall = std::dynamic_pointer_cast<MagneticBall>(pinball);
	// Project torque onto the Z-axis
	
	float length = sqrt(torque.x() * torque.x() + torque.y() * torque.y() + torque.z() * torque.z());

	float projectedTorqueMagnitude = length;
	
	// 1. Get the inertia from the magnetic ball
	float inertia = magneticBall->calculate_inertia();
	
	// 2. Calculate angular acceleration from torque and inertia
	float angularAcceleration = projectedTorqueMagnitude / inertia;
	
	// 3. Update angular speed based on this acceleration
	float oldAngularSpeed = node->getAngularSpeed();
	float newAngularSpeed = oldAngularSpeed + angularAcceleration * delta;
	
	float damping = 0.1f;
	newAngularSpeed *= damping;
	node->setAngularSpeed(newAngularSpeed);
	
	// 5. Calculate the rotation angle based on the updated angular speed
	float rotationAngle = newAngularSpeed * delta;
	
	// 6. Create the rotation and apply it (around Z-axis)
	msr::airlib::Quaternionr rotation
	= createQuaternionFromAxisAngle(axis, rotationAngle);
	msr::airlib::Quaternionr currentRotation = node->getRotationQuat();
	msr::airlib::Quaternionr newRotation = currentRotation * rotation;
	node->setRotationQuat(newRotation);
}
void Magnos::applyMagneticImpulse(float delta) {
	auto ironBall = std::dynamic_pointer_cast<MagneticBall>(pinball);
	
	// Combine magnetic forces
	auto forces = outerCoilSystem->combineFieldsOrForces(getWorldPosition3D());
	
	auto ironBallMagnets = innerMagnetSystem->getAttachedEntities();
	auto middleRingMagnets = middleMagnetSystem->getAttachedEntities();
	auto outerRingMagnets = outerMagnetSystem->getAttachedEntities();

	// Initialize total forces to zero
	msr::airlib::Vector3r ironBallTotalForce(0, 0, 0);
	msr::airlib::Vector3r middleRingTotalForce(0, 0, 0);
	
	for (const auto& ironBallMagnet : ironBallMagnets) {
		for (const auto& outerRingMagnet : outerRingMagnets) {
			// Calculate force between ironBallMagnet and outerRingMagnet
			msr::airlib::Vector3r forceOnIronBall = innerMagnetSystem->calculateForceDueToMagnet(
																								 msr::airlib::Vector3r(0, 0, 0), outerRingMagnet.position, ironBallMagnet.position, outerRingMagnet.polarity
																					);
			
			// Accumulate the force on the iron ball
			ironBallTotalForce += forceOnIronBall;
		}
	}
	
	for (const auto& middleRingMagnet : middleRingMagnets) {
		for (const auto& outerRingMagnet : outerRingMagnets) {
			// Calculate force between middleRingMagnet and outerRingMagnet
			msr::airlib::Vector3r forceOnMiddleRing =
			middleMagnetSystem->calculateForceDueToMagnet(
		    msr::airlib::Vector3r(0, 0, 0),
			outerRingMagnet.position,
			middleRingMagnet.position,
			outerRingMagnet.polarity);
			
			// Accumulate the forces on the iron ball and middle ring
			ironBallTotalForce += forceOnMiddleRing;
			middleRingTotalForce += -forceOnMiddleRing; // Opposite direction
		}
	}
	
	// Update the forces
	auto ironBallForces = ironBallTotalForce;
	auto middleForces = middleRingTotalForce;
	
	ironBallForces += forces;
	middleForces += forces;
	
	// Add Earth's magnetic force pointing north
	//		Vec3 earthMagneticForce = Vec3(-EARTH_MAGNETIC_FIELD_STRENGTH, -EARTH_MAGNETIC_FIELD_STRENGTH, 0) * SIMULATION_SCALE;
	
	
	// Assuming that forces are acting on some lever arm distance r from the rotation axis
	float r = 1; // This value should be set based on your system
	
	msr::airlib::Vector3r torqueInner = r * ironBallForces;
	applyTorqueAndRotate(innerNode, torqueInner, Settings::fixed_delta, msr::airlib::Vector3r(1, 0, 0));
	
	msr::airlib::Vector3r torqueMiddle = r * middleForces;
	applyTorqueAndRotate(middleNode, torqueMiddle, Settings::fixed_delta, msr::airlib::Vector3r(0, 1, 0));
}

AlternatorSystem& Magnos::getAlternatorSystem() { return alternator; }
CoilSystem& Magnos::getCoilSystem() { return *outerCoilSystem; }

void Magnos::init() {
	
	// Create the three gimbal rings using the given parameters
	outerNode = std::make_shared<Node>();
	middleNode = std::make_shared<Node>();
	innerNode = std::make_shared<Node>();
	
	// Outer Node Magnets with NORTH polarity
	outerNode->addChild(outerCoilSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::NORTH, MagnetPolarity::NORTH));
	outerNode->addChild(outerCoilSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH));
	outerNode->addChild(outerCoilSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::EAST, MagnetPolarity::NORTH));
	outerNode->addChild(outerCoilSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::WEST, MagnetPolarity::SOUTH));
	
	outerNode->addChild(alternator.attach(outerRingRadius + 0.0044f, MagnetDirection::NORTHEAST, MagnetPolarity::NORTH));
	
	outerNode->addChild(alternator.attach(outerRingRadius + 0.0044f, MagnetDirection::SOUTHEAST, MagnetPolarity::SOUTH));
	
	outerNode->addChild(alternator.attach(outerRingRadius + 0.0044f, MagnetDirection::NORTHWEST, MagnetPolarity::NORTH));
	
	outerNode->addChild(alternator.attach(outerRingRadius + 0.0044f, MagnetDirection::SOUTHWEST, MagnetPolarity::SOUTH));
	
	
	outerNode->addChild(outerMagnetSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::NORTHEAST, MagnetPolarity::SOUTH));
	outerNode->addChild(outerMagnetSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::NORTHWEST, MagnetPolarity::NORTH));
	outerNode->addChild(outerMagnetSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::SOUTHEAST, MagnetPolarity::SOUTH));
	outerNode->addChild(outerMagnetSystem->attach(outerRingRadius + 0.0024f, MagnetDirection::SOUTHWEST, MagnetPolarity::NORTH));

	middleNode->addChild(middleMagnetSystem->attach(middleRingRadius + 0.0016f, MagnetDirection::WEST, MagnetPolarity::SOUTH));
	middleNode->addChild(middleMagnetSystem->attach((baseDistanceOffset - 0.01f) + 0.0016f, MagnetDirection::EAST, MagnetPolarity::NORTH));
	
	innerNode->addChild(innerMagnetSystem->attach(innerRingRadius + 0.0016f, MagnetDirection::NORTH, MagnetPolarity::NORTH));
	innerNode->addChild(innerMagnetSystem->attach(innerRingRadius + 0.0016f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH));
	
	this->addChild(outerNode);
	outerNode->addChild(middleNode);
	middleNode->addChild(innerNode);
}


void Magnos::loadData(int id){
	outerCoilSystem =
	std::make_unique<CoilSystem>(id,
								 1.5f,
								 1.0f, // resistance
								 1.0f, // current
								 120); // turns
}

void Magnos::attachPinball() {
	
	pinball = std::make_shared<IronBall>(0.04f);
	
	innerNode->addChild(pinball);
}

