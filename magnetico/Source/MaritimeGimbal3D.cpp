#include "MaritimeGimbal3D.h"

ax::Mesh* MaritimeGimbal3D::createPole(float height, float radius, int segments) {
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texs;
	std::vector<unsigned short> indices;
	
	float angleStep = 2.0 * M_PI / segments;
	for (int i = 0; i <= segments; i++) {
		float angle = i * angleStep;
		positions.push_back(radius * cosf(angle));
		positions.push_back(-height / 2.0f);
		positions.push_back(radius * sinf(angle));
		
		positions.push_back(radius * cosf(angle));
		positions.push_back(height / 2.0f);
		positions.push_back(radius * sinf(angle));
		
		// Normals (roughly, since we aren't normalizing for simplicity)
		normals.push_back(cosf(angle));
		normals.push_back(0.0f);
		normals.push_back(sinf(angle));
		
		normals.push_back(cosf(angle));
		normals.push_back(0.0f);
		normals.push_back(sinf(angle));
		
		// Texture coords
		texs.push_back((float)i / segments);
		texs.push_back(0.0f);
		
		texs.push_back((float)i / segments);
		texs.push_back(1.0f);
		
		if (i < segments) {
			int offset = i * 2;
			indices.push_back(offset);
			indices.push_back(offset + 1);
			indices.push_back(offset + 2);
			
			indices.push_back(offset + 1);
			indices.push_back(offset + 3);
			indices.push_back(offset + 2);
		}
	}
	
	ax::IndexArray indexArray;
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	return ax::Mesh::create(positions, normals, texs, indexArray);
}
void MaritimeGimbal3D::addRodsToIronBall(IronBall* ball, float rodLength, float rodRadius) {
	auto northRod = ax::MeshRenderer::create();
	auto southRod = ax::MeshRenderer::create();
	auto eastRod = ax::MeshRenderer::create();
	auto westRod = ax::MeshRenderer::create();
	
	auto rodMesh1 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
	auto rodMesh2 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
	auto rodMesh3 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
	auto rodMesh4 = createPole(rodLength, rodRadius, 20); // 20 segments for the rod
	
	rodMesh1->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	rodMesh2->setMaterial(rodMesh1->getMaterial());
	rodMesh3->setMaterial(rodMesh1->getMaterial());
	rodMesh4->setMaterial(rodMesh1->getMaterial());
	
	northRod->addMesh(rodMesh1);
	southRod->addMesh(rodMesh2);
	eastRod->addMesh(rodMesh3);
	westRod->addMesh(rodMesh4);
	
	northRod->setTexture("gray.jpg");
	southRod->setTexture("gray.jpg");
	eastRod->setTexture("gray.jpg");
	westRod->setTexture("gray.jpg");
	
	float radius = 0.005f;
	// Positioning the rods
	northRod->setPosition3D(ax::Vec3(0, rodLength / 2 + radius, 0));
	southRod->setPosition3D(ax::Vec3(0, -(rodLength / 2 + radius), 0));
	eastRod->setPosition3D(ax::Vec3(rodLength / 2 + radius, 0, 0));
	westRod->setPosition3D(ax::Vec3(-(rodLength / 2 + radius), 0, 0));
	
	ball->addChild(northRod);
	ball->addChild(southRod);
	ball->addChild(eastRod);
	ball->addChild(westRod);
}

// Helper function to convert degrees to radians
float MaritimeGimbal3D::degToRad(float degrees) {
	return degrees * (M_PI / 180.0f);
}

// Helper function to convert radians to degrees
float MaritimeGimbal3D::radToDeg(float radians) {
	return radians * (180.0f / M_PI);
}


// This function returns a direction vector given an angle from the North (positive Z-axis)
ax::Vec3 MaritimeGimbal3D::directionFromAngle(float angleInDegrees) {
	float rad = degToRad(angleInDegrees);
	return ax::Vec3(cos(rad), sin(rad), 0);
}


void MaritimeGimbal3D::addPoles(ax::Node* node, float innerRadius, float distance, float poleRadius, const ax::Vec3& direction) {
	auto pole1 = ax::MeshRenderer::create();
	auto pole2 = ax::MeshRenderer::create();
	
	auto poleMesh1 = createPole(distance, poleRadius, 20);
	auto poleMesh2 = createPole(distance, poleRadius, 20);
	
	pole1->addMesh(poleMesh1);
	pole2->addMesh(poleMesh2);
	
	poleMesh1->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	poleMesh2->setMaterial(poleMesh1->getMaterial());
	
	pole1->setTexture("gray.jpg");
	pole2->setTexture("gray.jpg");
	
	// Set position based on direction
	ax::Vec3 polePosition1 = direction * (innerRadius + distance / 2);
	ax::Vec3 polePosition2 = -direction * (innerRadius + distance / 2);
	
	pole1->setPosition3D(polePosition1);
	pole2->setPosition3D(polePosition2);
	
	// Rotate horizontally if direction is East-West (i.e., along Z-axis)
	if(direction == ax::Vec3(1, 0, 0)) {
		pole1->setRotation3D(ax::Vec3(0, 0, 90));  // Rotate 90 degrees about Y-axis
		pole2->setRotation3D(ax::Vec3(0, 0, 90));  // Rotate 90 degrees about Y-axis
	}
	
	
	node->addChild(pole1);
	node->addChild(pole2);
}
void MaritimeGimbal3D::addPoles(ax::Node* node, float innerRadius, float distance, float poleRadius, float angleFromNorth) {
	auto pole1 = ax::MeshRenderer::create();
	auto pole2 = ax::MeshRenderer::create();
	
	auto poleMesh1 = createPole(distance, poleRadius, 20);
	auto poleMesh2 = createPole(distance, poleRadius, 20);
	
	pole1->addMesh(poleMesh1);
	pole2->addMesh(poleMesh2);
	
	poleMesh1->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	poleMesh2->setMaterial(poleMesh1->getMaterial());
	
	pole1->setTexture("gray.jpg");
	pole2->setTexture("gray.jpg");
	
	ax::Vec3 direction = directionFromAngle(angleFromNorth);
	
	ax::Vec3 polePosition1 = direction * (innerRadius + distance / 2);
	ax::Vec3 polePosition2 = -direction * (innerRadius + distance / 2);
	
	pole1->setPosition3D(polePosition1);
	pole2->setPosition3D(polePosition2);
	
	if(fabs(direction.x) < fabs(direction.y)) {
		pole1->setRotation3D(ax::Vec3(0, 0, 45));  // Rotate 90 degrees about Y-axis
		pole2->setRotation3D(ax::Vec3(0, 0, 45));  // Rotate 90 degrees about Y-axis
	} else {
		pole1->setRotation3D(ax::Vec3(0, 0, 135));  // Rotate 90 degrees about Y-axis
		pole2->setRotation3D(ax::Vec3(0, 0, 135));  // Rotate 90 degrees about Y-axis
	}
	
	node->addChild(pole1);
	node->addChild(pole2);
}

ax::Mesh* MaritimeGimbal3D::createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments) {
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texs;
	std::vector<unsigned short> indices;
	
	for (int m = 0; m <= majorSegments; m++) {
		float phi = 2.0 * M_PI * (float)m / (float)majorSegments;
		float cosPhi = cosf(phi);
		float sinPhi = sinf(phi);
		
		for (int n = 0; n <= minorSegments; n++) {
			float theta = 2.0 * M_PI * (float)n / (float)minorSegments;
			float cosTheta = cosf(theta);
			float sinTheta = sinf(theta);
			
			// Vertex positions
			float x = (majorRadius + minorRadius * cosTheta) * cosPhi;
			float y = (majorRadius + minorRadius * cosTheta) * sinPhi;
			float z = minorRadius * sinTheta;
			
			// Normals
			float nx = cosPhi * cosTheta;
			float ny = sinPhi * cosTheta;
			float nz = sinTheta;
			
			// Texture coordinates
			float u = (float)m / (float)majorSegments;
			float v = (float)n / (float)minorSegments;
			
			positions.push_back(x);
			positions.push_back(y);
			positions.push_back(z);
			
			normals.push_back(nx);
			normals.push_back(ny);
			normals.push_back(nz);
			
			texs.push_back(u);
			texs.push_back(v);
		}
	}
	
	// Indices
	for (int m = 0; m < majorSegments; m++) {
		for (int n = 0; n < minorSegments; n++) {
			int first = (m * (minorSegments + 1)) + n;
			int second = first + minorSegments + 1;
			int nextFirst = (m * (minorSegments + 1)) + ((n + 1) % (minorSegments + 1));
			int nextSecond = nextFirst + minorSegments + 1;
			
			indices.push_back(first);
			indices.push_back(second);
			indices.push_back(nextFirst);
			
			indices.push_back(second);
			indices.push_back(nextSecond);
			indices.push_back(nextFirst);
		}
	}
	
	ax::IndexArray indexArray;
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	return ax::Mesh::create(positions, normals, texs, indexArray);
}

ax::Mesh* MaritimeGimbal3D::createFlatDisk(float radius, float thickness, int majorSegments, int minorSegments) {
	// A flat disk is essentially a very flat torus
	return createTorus(radius + thickness / 2, thickness / 2, majorSegments, minorSegments);
}

ax::Vec3 MaritimeGimbal3D::rotateAroundAxis(const ax::Vec3& point, const ax::Vec3& axis, float angle) {
	if (axis == ax::Vec3(1, 0, 0)) { // Rotate around X
		return ax::Vec3(
						point.x,
						point.y * cosf(angle) - point.z * sinf(angle),
						point.y * sinf(angle) + point.z * cosf(angle)
						);
	} else if (axis == ax::Vec3(0, 1, 0)) { // Rotate around Y
		return ax::Vec3(
						point.x * cosf(angle) + point.z * sinf(angle),
						point.y,
						-point.x * sinf(angle) + point.z * cosf(angle)
						);
	} else if (axis == ax::Vec3(0, 0, 1)) { // Rotate around Z
		return ax::Vec3(
						point.x * cosf(angle) - point.y * sinf(angle),
						point.x * sinf(angle) + point.y * cosf(angle),
						point.z
						);
	} else {
		// No rotation
		return point;
	}
}

ax::Vec3 MaritimeGimbal3D::calculateMagneticFieldAt(const ax::Vec3& position) {
	ax::Vec3 totalB(0, 0, 0);  // Initialize the total magnetic field to zero
	
	auto ironBall = dynamic_cast<MagneticBall*>(pinball);
	
	float ballMagneticMoment = ironBall->get_magnetization() * innerNode->getAngularSpeed();
	
	auto& magnets = alternator.getAttachedEntities();
	for (auto& attachedMagnet : magnets) {
		// Calculate vector pointing from magnet to the position
		ax::Vec3 r = position - attachedMagnet.position;
		float distance = r.length();
		
		// Normalize the vector r
		r.normalize();
		
		// Calculate the magnetic moment (simplified) for the attached magnet
		float m = ballMagneticMoment * ironBall->calculate_volume();
		
		// Biot-Savart approximation for magnetic field due to a small magnet
		ax::Vec3 B = (MU_0 / (4.0f * M_PI)) * (m / std::pow(distance, 3)) * r;
		
		// Sum up the contributions
		totalB += B;
	}
	
	
	return totalB;
}

float MaritimeGimbal3D::calculateFluxThroughCoil(const ax::Vec3& B, float coilArea, int coilTurns) {
	return coilTurns * coilArea * B.length();
}

void MaritimeGimbal3D::calculateFlux(CoilEntity::AttachedEntity& coil, const ax::Vec3& position) {
	auto ironBall = dynamic_cast<MagneticBall*>(pinball);
	float ballMagneticMoment = ironBall->get_magnetization() * ironBall->calculate_volume();
	
	ax::Vec3 r = position - coil.position;
	float distance = r.length();
	r.normalize();
	
	// Magnetic field at the coil's position due to the ball
	ax::Vec3 B = (MU_0 / (4.0f * M_PI)) * (ballMagneticMoment / std::pow(distance, 3)) * r;
	
	float flux = B.length() * coil.area; // assuming coil's face is perpendicular to B
	
	// Calculate the flux through the coil due to this magnetic field
	coil.flux = flux;
	
}

float MaritimeGimbal3D::calculateCoilEMF(const CoilEntity::AttachedEntity& coil, float delta) {
	// Change in flux
	float deltaPhi = coil.flux - coil.previousFlux;
	
	// EMF for this coil using Faraday's law
	return -coil.turns * (deltaPhi / delta);
}

void MaritimeGimbal3D::update(float) {
	//global_delta = dt;
	
	//desired_voltage_per_second = desired_voltage * dt;
	int updates = 0;
	
	do{
		
		middleMagnetSystem->update();
		innerMagnetSystem->update();
		outerCoilSystem->update();
		alternator.update();
		
		auto ironBall = dynamic_cast<MagneticBall*>(pinball);
		
		applyMagneticImpulse(global_delta);
		
		// Compute flux for all coils
		float totalEMF = 0.0f;
		auto& outerEntities = middleMagnetSystem->getAttachedEntities();
		auto& innerEntities = innerMagnetSystem->getAttachedEntities();
		
		auto& coils = alternator.getAttachedEntities();
		
		for (auto& coil : coils) {
			// Reset the coil's flux before summing from all entities
			coil.previousFlux = coil.flux;
			coil.flux = 0.0f;
			
			for (auto& entity : outerEntities) {
				// Compute flux for the current coil based on each entity's position
				calculateFlux(coil, entity.position);
			}
			
			// Compute the EMF for the coil after accounting for all entities
			totalEMF += calculateCoilEMF(coil, global_delta);
		}
		// Compute total induced EMF in the alternator
		alternator.emf = totalEMF;
		
		
		outerCoilSystem->update(alternator.emf, global_delta);
		
		updates++;
	} while(updates < Settings::cycles_per_collection);
	
}
float MaritimeGimbal3D::calculateEffectiveArea(const ax::Quaternion& rotation) {
	// Original dimensions of the innerNode
	const float length = 1.0f;  // Replace with actual value
	const float width = 1.0f;   // Replace with actual value
	float A_original = length * width;
	
	// Assuming the normal vector of the unrotated innerNode is (0, 0, 1)
	ax::Vec3 unrotatedNormal(0, 0, 1);
	ax::Vec3 rotatedNormal = rotation * unrotatedNormal;  // Apply quaternion rotation to the normal
	
	// Assuming the magnetic field direction is vertical (0, 0, 1)
	ax::Vec3 magneticFieldDirection(0, 0, 1);
	
	// Compute the cosine of the angle between the rotated normal and the magnetic field direction
	float cosTheta = rotatedNormal.dot(magneticFieldDirection);
	
	return A_original * cosTheta;
}

void MaritimeGimbal3D::applyTorqueAndRotate(CustomNode* node, const ax::Vec3& torque, float delta, ax::Vec3 axis) {
	MagneticBall* magneticBall = dynamic_cast<MagneticBall*>(pinball);
	// Project torque onto the Z-axis
	float projectedTorqueMagnitude = torque.length();
	
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
	ax::Quaternion rotation;
	ax::Quaternion::createFromAxisAngle(axis, rotationAngle, &rotation);
	ax::Quaternion currentRotation = node->getRotationQuat();
	ax::Quaternion newRotation = currentRotation * rotation;
	node->setRotationQuat(newRotation);
	
}
void MaritimeGimbal3D::applyMagneticImpulse(float delta) {
	auto ironBall = dynamic_cast<MagneticBall*>(pinball);
	
	// Combine magnetic forces
	auto forces = outerCoilSystem->combineFieldsOrForces();
	
	auto ironBallMagnets = innerMagnetSystem->getAttachedEntities();
	auto middleRingMagnets = middleMagnetSystem->getAttachedEntities();
	
	ax::Vec3 ironBallTotalForce(0, 0, 0);
	ax::Vec3 middleRingTotalForce(0, 0, 0);
	
	for (const auto& ironBallMagnet : ironBallMagnets) {
		for (const auto& middleRingMagnet : middleRingMagnets) {
			// Force on ironBallMagnet due to middleRingMagnet
			ax::Vec3 forceOnIronBall =  innerMagnetSystem->calculateForceDueToMagnet(middleRingMagnet.position, ironBallMagnet.position, middleRingMagnet.polarity);
			ironBallTotalForce += forceOnIronBall;
			
			// Force on middleRingMagnet due to ironBallMagnet (opposite direction)
			ax::Vec3 forceOnMiddleRing = -forceOnIronBall;
			middleRingTotalForce += forceOnMiddleRing;
		}
	}
	
	auto ironBallForces = ironBallTotalForce;
	auto middleForces = middleRingTotalForce;
	
	ironBallForces += forces;
	middleForces += forces;
	
	// Add Earth's magnetic force pointing north
	//		Vec3 earthMagneticForce = Vec3(-EARTH_MAGNETIC_FIELD_STRENGTH, -EARTH_MAGNETIC_FIELD_STRENGTH, 0) * SIMULATION_SCALE;
	
	
	// Assuming that forces are acting on some lever arm distance r from the rotation axis
	float r = 1; // This value should be set based on your system
	
	ax::Vec3 torqueInner = r * ironBallForces;
	applyTorqueAndRotate(innerNode, torqueInner, global_delta, ax::Vec3(1, 0, 0));
	
	ax::Vec3 torqueMiddle = r * middleForces;
	applyTorqueAndRotate(middleNode, torqueMiddle, global_delta, ax::Vec3(0, 1, 0));
}

AlternatorSystem& MaritimeGimbal3D::getAlternatorSystem() { return alternator; }
CoilSystem& MaritimeGimbal3D::getCoilSystem() { return *outerCoilSystem; }

bool MaritimeGimbal3D::init() {
	if (!ax::Node::init()) return false;
	
	float baseThicknessOffset = 0.0012f;  // Base thickness offset for the gimbal rings
	
	// Create the three gimbal rings using the given parameters
	innerRing = createFlatDisk(innerRingRadius, baseThicknessOffset, 40, 20);
	middleRing = createFlatDisk(middleRingRadius, baseThicknessOffset * (2.0f / 3.0f), 40, 20);
	outerRing = createFlatDisk(outerRingRadius, baseThicknessOffset, 40, 20);
	
	outerNode = CustomNode::create();
	middleNode = CustomNode::create();
	innerNode = CustomNode::create();
	
	outerNode->addMesh(outerRing);
	middleNode->addMesh(middleRing);
	innerNode->addMesh(innerRing);
	
	outerRing->setTexture("gold.jpg");
	middleRing->setTexture("gold.jpg");
	innerRing->setTexture("gold.jpg");
	
	// Outer Node Magnets with NORTH polarity
	outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::NORTH, MagnetPolarity::NORTH);
	outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH);
	
	outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::EAST, MagnetPolarity::NORTH);
	outerCoilSystem->attachToDisk(outerNode, outerRingRadius + 0.0024f, MagnetDirection::WEST, MagnetPolarity::SOUTH);
	
	
	alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::NORTHEAST, MagnetPolarity::SOUTH);
	
	alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::SOUTHEAST, MagnetPolarity::SOUTH);
	
	alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::NORTHWEST, MagnetPolarity::SOUTH);
	
	alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::SOUTHWEST, MagnetPolarity::SOUTH);
	
	
	alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::FRONT, MagnetPolarity::SOUTH);
	
	
	alternator.attachToDisk(outerNode, outerRingRadius + 0.0044f, MagnetDirection::BACK, MagnetPolarity::SOUTH);
	
	
	middleMagnetSystem->attachToDisk(middleNode, middleRingRadius + 0.0016f, MagnetDirection::WEST, MagnetPolarity::SOUTH);
	middleMagnetSystem->attachToDisk(middleNode, (baseDistanceOffset - 0.01f) + 0.0016f, MagnetDirection::EAST, MagnetPolarity::NORTH);
	
	innerMagnetSystem->attachToDisk(innerNode, innerRingRadius + 0.0016f, MagnetDirection::NORTH, MagnetPolarity::NORTH);
	innerMagnetSystem->attachToDisk(innerNode, innerRingRadius + 0.0016f, MagnetDirection::SOUTH, MagnetPolarity::SOUTH);
	
	
	this->outerNode->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	this->middleNode->setMaterial(this->outerNode->getMaterial());
	this->innerNode->setMaterial(this->outerNode->getMaterial());
	
	this->addChild(outerNode);
	outerNode->addChild(middleNode);
	middleNode->addChild(innerNode);
	
	this->addPoles(outerNode, middleRingRadius, (outerRingRadius - middleRingRadius), 0.001f, ax::Vec3(0, 1, 0));  // Poles for the outer ring, along Y-axis
	this->addPoles(middleNode, innerRingRadius, (middleRingRadius - innerRingRadius), 0.001f, ax::Vec3(1, 0, 0)); // Poles for the middle ring, along Z-axis
	// Adding poles:
	this->addPoles(innerNode, 0.04f, (innerRingRadius - 0.04f), 0.001f, 45);  // NE
	this->addPoles(innerNode, 0.04f, (innerRingRadius - 0.04f), 0.001f, 135); // SE
	this->scheduleUpdate();
	
	
	return true;
}

void MaritimeGimbal3D::attachPinball(ax::Scene* scene) {
	
	pinball = IronBall::create(0.04f);
	
	//scene->getPhysics3DWorld()->addPhysics3DObject(dynamic_cast<MagneticBall*>(pinball)->getPhysics3DRigidBody());
	
	innerNode->addChild(pinball);
	
	middleMagnetSystem->update();
	innerMagnetSystem->update();
	outerCoilSystem->update();
	alternator.update();
	
	
	// Compute flux for all coils
	float totalEMF = 0.0f;
	auto& outerEntities = middleMagnetSystem->getAttachedEntities();
	auto& innerEntities = innerMagnetSystem->getAttachedEntities();
	
	auto& coils = alternator.getAttachedEntities();
	
	for (auto& coil : coils) {
		// Reset the coil's flux before summing from all entities
		coil.previousFlux = coil.flux;
		coil.flux = 0.0f;
		
		for (auto& entity : outerEntities) {
			// Compute flux for the current coil based on each entity's position
			calculateFlux(coil, entity.position);
		}
		
		coil.previousFlux = coil.flux;
		
	}
}

