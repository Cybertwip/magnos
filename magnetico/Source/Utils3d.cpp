#include "Utils3d.h"

ax::Mesh* createCube(float dimension){
	
	// Vertices for a cube
	std::vector<float> positions = {
		-dimension, -dimension, dimension,
		dimension, -dimension, dimension,
		dimension,  dimension, dimension,
		-dimension,  dimension, dimension,
		-dimension, -dimension, -dimension,
		dimension, -dimension, -dimension,
		dimension,  dimension, -dimension,
		-dimension,  dimension, -dimension
	};
	
	// Normals for the cube (not normalized, for the sake of simplicity)
	std::vector<float> normals = {
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1
	};
	
	// Texture coordinates for the cube
	std::vector<float> texs = {
		0, 0,
		1, 0,
		1, 1,
		0, 1,
		0, 0,
		1, 0,
		1, 1,
		0, 1
	};
	
	// Indices for the cube
	std::vector<unsigned short> indices = {
		0, 1, 2, 0, 2, 3,   // Front face
		5, 4, 7, 5, 7, 6,   // Back face
		4, 0, 3, 4, 3, 7,   // Left face
		1, 5, 6, 1, 6, 2,   // Right face
		4, 5, 1, 4, 1, 0,   // Bottom face
		3, 2, 6, 3, 6, 7    // Top face
	};
	
	ax::IndexArray indexArray;
	
	for(auto index : indices){
		indexArray.emplace_back(index);
	}
	
	// Create mesh and attach to Sprite3D
	return ax::Mesh::create(positions, normals, texs, indexArray);
}

ax::Mesh* createCuboid(float width, float height, float depth) {
	// Calculate half-dimensions for convenience
	float halfWidth = width;
	float halfHeight = height;
	float halfDepth = depth;
	
	// Vertices for a cuboid
	std::vector<float> positions = {
		-halfWidth, -halfHeight, halfDepth,
		halfWidth, -halfHeight, halfDepth,
		halfWidth, halfHeight, halfDepth,
		-halfWidth, halfHeight, halfDepth,
		-halfWidth, -halfHeight, -halfDepth,
		halfWidth, -halfHeight, -halfDepth,
		halfWidth, halfHeight, -halfDepth,
		-halfWidth, halfHeight, -halfDepth
	};
	
	// Normals for the cuboid (not normalized, for the sake of simplicity)
	std::vector<float> normals = {
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1
	};
	
	// Texture coordinates for the cuboid
	std::vector<float> texs = {
		0, 0,
		1, 0,
		1, 1,
		0, 1,
		0, 0,
		1, 0,
		1, 1,
		0, 1
	};
	
	// Indices for the cuboid
	std::vector<unsigned short> indices = {
		0, 1, 2, 0, 2, 3,   // Front face
		5, 4, 7, 5, 7, 6,   // Back face
		4, 0, 3, 4, 3, 7,   // Left face
		1, 5, 6, 1, 6, 2,   // Right face
		4, 5, 1, 4, 1, 0,   // Bottom face
		3, 2, 6, 3, 6, 7    // Top face
	};
	
	ax::IndexArray indexArray;
	
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	// Create mesh and attach to Sprite3D
	return ax::Mesh::create(positions, normals, texs, indexArray);
}
ax::Mesh* createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments) {
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

ax::Mesh* createFlatDisk(float radius, float thickness, int majorSegments, int minorSegments) {
	// A flat disk is essentially a very flat torus
	auto torus = createTorus(radius + thickness / 2, thickness / 2, majorSegments, minorSegments);
	
	torus->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	torus->setTexture("kitty.jpg");
	
	return torus;

}

// Function to create the car body
ax::Node* createCarBody(float carDimension) {
	ax::Mesh* carBodyMesh = createCuboid(carDimension, carDimension / 2, carDimension / 2);
	
	ax::MeshRenderer* carBody = ax::MeshRenderer::create();
	carBody->addMesh(carBodyMesh);
	carBody->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	carBody->setTexture("spidey.jpg");
	
	carBody->setOpacity(144);

	ax::Node* carBodyNode = ax::Node::create();
	carBodyNode->addChild(carBody);
	

	return carBodyNode;
}

// Function to create the car roof
ax::Node* createCarRoof(float carDimension, float roofHeight) {
	ax::Mesh* carRoofMesh = createCube(carDimension);
	// Adjust the roof mesh dimensions and position as needed
	
	ax::MeshRenderer* carRoof = ax::MeshRenderer::create();
	carRoof->addMesh(carRoofMesh);
	carRoof->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	carRoof->setTexture("black.jpg");
	
	ax::Node* carRoofNode = ax::Node::create();
	carRoofNode->setPosition3D(ax::Vec3(0, carDimension / 2 + roofHeight / 2, 0));
	carRoofNode->addChild(carRoof);
	
	return carRoofNode;
}

// Function to create the car windows
ax::Node* createCarWindows(float carDimension, float roofHeight) {
	ax::Mesh* carWindowsMesh1 = createCuboid(roofHeight, carDimension / 2 - roofHeight - carDimension / 2 * 0.25f, roofHeight * 0.01f);
	ax::Mesh* carWindowsMesh2 = createCuboid(roofHeight, carDimension / 2 - roofHeight - carDimension / 2 * 0.25f, roofHeight * 0.01f);
	// Adjust the window mesh dimensions and position as needed
	
	ax::MeshRenderer* carWindow1 = ax::MeshRenderer::create();
	ax::MeshRenderer* carWindow2 = ax::MeshRenderer::create();
	carWindow1->addMesh(carWindowsMesh1);
	carWindow1->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	carWindow1->setTexture("gray.jpg");
	carWindow1->setOpacity(200);

	carWindow2->addMesh(carWindowsMesh2);
	carWindow2->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	carWindow2->setTexture("gray.jpg");
	carWindow2->setOpacity(200);

	ax::Node* carWindowsNode = ax::Node::create();
	ax::Node* carWindowsNode1 = ax::Node::create();
	ax::Node* carWindowsNode2 = ax::Node::create();
	carWindowsNode1->setPosition3D(ax::Vec3(carDimension * 0.45f, carDimension / 2 - roofHeight, carDimension / 2));
	carWindowsNode2->setPosition3D(ax::Vec3(carDimension * 0.45f, carDimension / 2 - roofHeight, -carDimension / 2));

	carWindowsNode1->addChild(carWindow1);
	carWindowsNode2->addChild(carWindow2);
	
	carWindowsNode->addChild(carWindowsNode1);
	carWindowsNode->addChild(carWindowsNode2);

	return carWindowsNode;
}

ax::Node* createCarWithWheels(float carDimension, float wheelRadius, float wheelWidth) {
	// Create the car body, roof, and windows as separate meshes
	ax::Node* carBody = createCarBody(carDimension);
	//ax::Node* roof = createCarRoof(carDimension, 1); // You may need to define roofHeight
	ax::Node* windows = createCarWindows(carDimension, 0.25f);
	
	// Create four wheels and attach them to the car body
	ax::Mesh* frontLeftWheelMesh = createFlatDisk(wheelRadius, wheelWidth, 40, 20);
	ax::Mesh* frontRightWheelMesh = createFlatDisk(wheelRadius, wheelWidth, 40, 20);
	ax::Mesh* rearLeftWheelMesh = createFlatDisk(wheelRadius, wheelWidth, 40, 20);
	ax::Mesh* rearRightWheelMesh = createFlatDisk(wheelRadius, wheelWidth, 40, 20);
	
	ax::MeshRenderer* frontLeftWheel = ax::MeshRenderer::create();
	ax::MeshRenderer* frontRightWheel = ax::MeshRenderer::create();
	ax::MeshRenderer* rearLeftWheel = ax::MeshRenderer::create();
	ax::MeshRenderer* rearRightWheel = ax::MeshRenderer::create();
	
	frontLeftWheel->addMesh(frontLeftWheelMesh);
	frontRightWheel->addMesh(frontRightWheelMesh);
	rearLeftWheel->addMesh(rearLeftWheelMesh);
	rearRightWheel->addMesh(rearRightWheelMesh);
	
	// Position the wheels relative to the car body
	// You can adjust these positions as needed to align the wheels correctly
	frontLeftWheel->setPosition3D(ax::Vec3(-carDimension + wheelRadius, -carDimension / 2, -carDimension / 2 - wheelRadius));
	frontRightWheel->setPosition3D(ax::Vec3(carDimension - wheelRadius, -carDimension / 2, carDimension / 2 + wheelRadius));
	rearLeftWheel->setPosition3D(ax::Vec3(-carDimension + wheelRadius, -carDimension / 2, carDimension / 2 + wheelRadius));
	rearRightWheel->setPosition3D(ax::Vec3(carDimension - wheelRadius, -carDimension / 2, -carDimension / 2 - wheelRadius));
	
	// Create a container node for the entire car (body, roof, windows, and wheels)
	ax::Node* carNode = ax::Node::create();
	carNode->addChild(carBody);
	carNode->addChild(frontLeftWheel);
	carNode->addChild(frontRightWheel);
	carNode->addChild(rearLeftWheel);
	carNode->addChild(rearRightWheel);
	
	// Attach roof and windows to the car body
	//carBody->addChild(roof);
	carBody->addChild(windows);
	
	return carNode;
}
