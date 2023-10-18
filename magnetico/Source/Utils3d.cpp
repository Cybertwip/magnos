#include "Utils3d.h"
#include "CustomNode.h"

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

ax::Mesh* createCylinder(float radius, float height, int numSegments) {
	// Initialize vectors to store vertices, normals, texture coordinates, and indices
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texs;
	std::vector<unsigned short> indices;
	
	// Create vertices for the top and bottom faces
	for (int i = 0; i < numSegments; ++i) {
		float angle = 2.0 * M_PI * static_cast<float>(i) / static_cast<float>(numSegments);
		float x = radius * std::cos(angle);
		float y = radius * std::sin(angle);
		
		// Top face
		positions.push_back(x);
		positions.push_back(y);
		positions.push_back(0.5 * height);
		
		normals.push_back(0.0f);
		normals.push_back(0.0f);
		normals.push_back(1.0f);
		
		texs.push_back(0.5f + 0.5f * x / radius);
		texs.push_back(0.5f + 0.5f * y / radius);
		
		// Bottom face
		positions.push_back(x);
		positions.push_back(y);
		positions.push_back(-0.5 * height);
		
		normals.push_back(0.0f);
		normals.push_back(0.0f);
		normals.push_back(-1.0f);
		
		texs.push_back(0.5f + 0.5f * x / radius);
		texs.push_back(0.5f + 0.5f * y / radius);
	}
	
	// Create vertices for the side surface
	for (int i = 0; i < numSegments; ++i) {
		float angle = 2.0 * M_PI * static_cast<float>(i) / static_cast<float>(numSegments);
		float x = radius * std::cos(angle);
		float y = radius * std::sin(angle);
		
		// Side vertices
		positions.push_back(x);
		positions.push_back(y);
		positions.push_back(0.5 * height);
		
		normals.push_back(x / radius);
		normals.push_back(y / radius);
		normals.push_back(0.0f);
		
		texs.push_back(static_cast<float>(i) / static_cast<float>(numSegments));
		texs.push_back(0.0f);
		
		positions.push_back(x);
		positions.push_back(y);
		positions.push_back(-0.5 * height);
		
		normals.push_back(x / radius);
		normals.push_back(y / radius);
		normals.push_back(0.0f);
		
		texs.push_back(static_cast<float>(i) / static_cast<float>(numSegments));
		texs.push_back(1.0f);
	}
	
	// Create indices for the top, bottom, and side faces
	for (int i = 0; i < numSegments; ++i) {
		int next = (i + 1) % numSegments;
		
		// Top face indices
		indices.push_back(i);
		indices.push_back(i + numSegments);
		indices.push_back(next);
		
		// Bottom face indices
		indices.push_back(i + numSegments * 2);
		indices.push_back(i + numSegments * 3);
		indices.push_back(next + numSegments * 2);
		
		// Side face indices
		indices.push_back(i + numSegments * 2);
		indices.push_back(i + numSegments * 3);
		indices.push_back(next + numSegments * 2);
		
		indices.push_back(i);
		indices.push_back(i + numSegments);
		indices.push_back(next + numSegments);
		
		indices.push_back(i + numSegments);
		indices.push_back(next + numSegments);
		indices.push_back(next);
		
	}
	
	ax::IndexArray indexArray;
	
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	// Create the mesh and attach to Sprite3D
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
	// Calculate texture coordinates based on the size of each face
	float texCoordX = 1;
	float texCoordY = 1;
	
	// Texture coordinates for the cuboid
	std::vector<float> texs = {
		0, 0,
		texCoordX, 0,
		texCoordX, texCoordY,
		0, texCoordY,
		0, 0,
		texCoordX, 0,
		texCoordX, texCoordY,
		0, texCoordY
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

ax::Mesh* createSphere(float radius, int segments, int rings) {
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texs;
	std::vector<unsigned short> indices;
	ax::IndexArray indexArray;
	
	// Generate sphere vertices, normals, texture coordinates, and indices
	for (int i = 0; i <= rings; ++i) {
		float v = static_cast<float>(i) / rings;
		float theta = v * M_PI;
		float sinTheta = sin(theta);
		float cosTheta = cos(theta);
		
		for (int j = 0; j <= segments; ++j) {
			float u = static_cast<float>(j) / segments;
			float phi = u * M_PI * 2;
			float sinPhi = sin(phi);
			float cosPhi = cos(phi);
			
			float x = cosPhi * sinTheta;
			float y = cosTheta;
			float z = sinPhi * sinTheta;
			
			positions.push_back(radius * x);
			positions.push_back(radius * y);
			positions.push_back(radius * z);
			
			normals.push_back(x);
			normals.push_back(y);
			normals.push_back(z);
			
			texs.push_back(u);
			texs.push_back(1.0f - v);
		}
	}
	
	// Generate sphere indices
	for (int i = 0; i < rings; ++i) {
		for (int j = 0; j < segments; ++j) {
			int nextRow = i + 1;
			int nextColumn = j + 1;
			
			indices.push_back(i * (segments + 1) + j);
			indices.push_back(nextRow * (segments + 1) + j);
			indices.push_back(nextRow * (segments + 1) + nextColumn);
			
			indices.push_back(i * (segments + 1) + j);
			indices.push_back(nextRow * (segments + 1) + nextColumn);
			indices.push_back(i * (segments + 1) + nextColumn);
		}
	}
	
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	// Create the mesh and return it
	return ax::Mesh::create(positions, normals, texs, indexArray);
}

ax::Mesh* createCapsule(float radius, float cylinderHeight, int sphereSegments, int sphereRings, int cylinderSegments) {
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texs;
	std::vector<unsigned short> indices;
	ax::IndexArray indexArray;
	
	// Create the top sphere
	for (int i = 0; i <= sphereRings; ++i) {
		float v = static_cast<float>(i) / sphereRings;
		float theta = v * M_PI;
		float sinTheta = sin(theta);
		float cosTheta = cos(theta);
		
		for (int j = 0; j <= sphereSegments; ++j) {
			float u = static_cast<float>(j) / sphereSegments;
			float phi = u * M_PI * 2;
			float sinPhi = sin(phi);
			float cosPhi = cos(phi);
			
			float x = cosPhi * sinTheta;
			float y = cosTheta;
			float z = sinPhi * sinTheta;
			
			positions.push_back(radius * x);
			positions.push_back(radius * y);
			positions.push_back(radius * z);
			
			normals.push_back(x);
			normals.push_back(y);
			normals.push_back(z);
			
			texs.push_back(u);
			texs.push_back(1.0f - v);
		}
	}
	
	// Create the bottom sphere
	for (int i = 0; i <= sphereRings; ++i) {
		float v = static_cast<float>(i) / sphereRings;
		float theta = v * M_PI;
		float sinTheta = sin(theta);
		float cosTheta = cos(theta);
		
		for (int j = 0; j <= sphereSegments; ++j) {
			float u = static_cast<float>(j) / sphereSegments;
			float phi = u * M_PI * 2;
			float sinPhi = sin(phi);
			float cosPhi = cos(phi);
			
			float x = cosPhi * sinTheta;
			float y = cosTheta;
			float z = sinPhi * sinTheta;
			
			positions.push_back(radius * x);
			positions.push_back(radius * y);
			positions.push_back(radius * z);
			
			normals.push_back(x);
			normals.push_back(y);
			normals.push_back(z);
			
			texs.push_back(u);
			texs.push_back(1.0f - v);
		}
	}
	
	// Create the cylindrical part
	for (int i = 0; i <= cylinderSegments; ++i) {
		float u = static_cast<float>(i) / cylinderSegments;
		float phi = u * M_PI * 2;
		float sinPhi = sin(phi);
		float cosPhi = cos(phi);
		
		for (int j = 0; j <= cylinderHeight; ++j) {
			float v = static_cast<float>(j) / cylinderHeight;
			
			float x = radius * cosPhi;
			float y = radius * sinPhi;
			float z = (v - 0.5) * 2 * radius;
			
			positions.push_back(x);
			positions.push_back(y);
			positions.push_back(z);
			
			normals.push_back(cosPhi);
			normals.push_back(sinPhi);
			normals.push_back(0.0f);
			
			texs.push_back(u);
			texs.push_back(v);
		}
	}
	
	// Create indices for the spheres
	int numSphereVertices = (sphereRings + 1) * (sphereSegments + 1);
	for (int i = 0; i < numSphereVertices - sphereSegments - 1; ++i) {
		if ((i + 1) % (sphereSegments + 1) != 0) {
			indices.push_back(i);
			indices.push_back(i + 1);
			indices.push_back(i + sphereSegments + 1);
			
			indices.push_back(i + sphereSegments + 1);
			indices.push_back(i + 1);
			indices.push_back(i + sphereSegments + 2);
		}
	}
	
	// Create indices for the cylindrical part
	int numCylinderVertices = (cylinderSegments + 1) * (cylinderHeight + 1);
	for (int i = numSphereVertices; i < numSphereVertices + numCylinderVertices - cylinderSegments - 1; ++i) {
		if ((i + 1) % (cylinderSegments + 1) != 0) {
			indices.push_back(i);
			indices.push_back(i + 1);
			indices.push_back(i + cylinderSegments + 1);
			
			indices.push_back(i + cylinderSegments + 1);
			indices.push_back(i + 1);
			indices.push_back(i + cylinderSegments + 2);
		}
	}
	
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	// Create the mesh for the capsule and return it
	return ax::Mesh::create(positions, normals, texs, indexArray);
}


ax::Mesh* createPlane(float width, float depth, int texRepeatX, int texRepeatY) {
	// Calculate half-dimensions for convenience
	float halfWidth = width / 2.0f;
	float halfDepth = depth / 2.0f;
	
	// Vertices for a plane
	std::vector<float> positions = {
		-halfWidth, 0.0f, halfDepth,
		halfWidth, 0.0f, halfDepth,
		halfWidth, 0.0f, -halfDepth,
		-halfWidth, 0.0f, -halfDepth,
	};
	
	// Normals for the plane (all pointing up)
	std::vector<float> normals = {
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
	};
	
	// Calculate texture coordinates based on the size of the plane
	float texCoordX = width * texRepeatX / 16.0f;
	float texCoordY = depth * texRepeatY / 16.0f;
	
	// Texture coordinates for the plane
	std::vector<float> texs = {
		0, 0,
		texCoordX, 0,
		texCoordX, texCoordY,
		0, texCoordY,
	};
	
	// Indices for the plane
	std::vector<unsigned short> indices = {
		0, 1, 2, 0, 2, 3,
	};
	
	ax::IndexArray indexArray;
	
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	// Create mesh and return it
	return ax::Mesh::create(positions, normals, texs, indexArray);
}

ax::Mesh* createHollowCuboid(float outerWidth, float outerHeight, float outerDepth, float innerWidth, float innerHeight, float innerDepth) {
	// Calculate half-dimensions for convenience
	float halfOuterWidth = outerWidth / 2.0f;
	float halfOuterHeight = outerHeight / 2.0f;
	float halfOuterDepth = outerDepth / 2.0f;
	float halfInnerWidth = innerWidth / 2.0f;
	float halfInnerHeight = innerHeight / 2.0f;
	float halfInnerDepth = innerDepth / 2.0f;
	
	// Calculate vertices for the hollow cuboid
	std::vector<float> positions = {
		// Outer vertices
		-halfOuterWidth, -halfOuterHeight, halfOuterDepth,
		halfOuterWidth, -halfOuterHeight, halfOuterDepth,
		halfOuterWidth, halfOuterHeight, halfOuterDepth,
		-halfOuterWidth, halfOuterHeight, halfOuterDepth,
		-halfOuterWidth, -halfOuterHeight, -halfOuterDepth,
		halfOuterWidth, -halfOuterHeight, -halfOuterDepth,
		halfOuterWidth, halfOuterHeight, -halfOuterDepth,
		-halfOuterWidth, halfOuterHeight, -halfOuterDepth,
		
		// Inner vertices
		-halfInnerWidth, -halfInnerHeight, halfInnerDepth,
		halfInnerWidth, -halfInnerHeight, halfInnerDepth,
		halfInnerWidth, halfInnerHeight, halfInnerDepth,
		-halfInnerWidth, halfInnerHeight, halfInnerDepth,
		-halfInnerWidth, -halfInnerHeight, -halfInnerDepth,
		halfInnerWidth, -halfInnerHeight, -halfInnerDepth,
		halfInnerWidth, halfInnerHeight, -halfInnerDepth,
		-halfInnerWidth, halfInnerHeight, -halfInnerDepth,
	};
	
	// Normals (assuming the outer faces have normals pointing outward and inner faces have normals pointing inward)
	std::vector<float> normals = {
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		
		// Inner normals (opposite of outer normals)
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
	};
	
	// Texture coordinates
	std::vector<float> texs = {
		// Outer texture coordinates (can be adjusted based on your needs)
		0, 0,
		1, 0,
		1, 1,
		0, 1,
		0, 0,
		1, 0,
		1, 1,
		0, 1,
		
		// Inner texture coordinates (can be adjusted based on your needs)
		0, 0,
		1, 0,
		1, 1,
		0, 1,
		0, 0,
		1, 0,
		1, 1,
		0, 1,
	};
	
	// Indices for the hollow cuboid
	std::vector<unsigned short> indices = {
		// Front outer face
		0, 1, 2, 0, 2, 3,
		// Back outer face
		5, 4, 7, 5, 7, 6,
		// Left outer face
		4, 0, 3, 4, 3, 7,
		// Right outer face
		1, 5, 6, 1, 6, 2,
		// Bottom outer face
		4, 5, 1, 4, 1, 0,
		// Top outer face
		3, 2, 6, 3, 6, 7,
		
		// Front inner face
		8, 9, 10, 8, 10, 11,
		// Back inner face
		13, 12, 15, 13, 15, 14,
		// Left inner face
		12, 8, 11, 12, 11, 15,
		// Right inner face
		9, 13, 14, 9, 14, 10,
		// Bottom inner face
		12, 13, 9, 12, 9, 8,
		// Top inner face
		11, 10, 14, 11, 14, 15,
	};
	
	ax::IndexArray indexArray;
	
	for (auto index : indices) {
		indexArray.emplace_back(index);
	}
	
	// Create mesh for the hollow cuboid
	ax::Mesh* cuboidMesh = ax::Mesh::create(positions, normals, texs, indexArray);
	
	return cuboidMesh;
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
	torus->setTexture("gray.jpg");
	
	return torus;
}

ax::Node* createRearSuspension(float width, float height, float depth) {
	// Create a cylinder for the rear suspension
	ax::Mesh* rearSuspensionMesh = createCuboid(width, height, depth);
	
	ax::MeshRenderer* rearSuspensionMeshRenderer = ax::MeshRenderer::create();
	
	rearSuspensionMeshRenderer->addMesh(rearSuspensionMesh);

	// Set material, texture, and other properties as needed
	rearSuspensionMeshRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	rearSuspensionMeshRenderer->setTexture("kitty.jpg");
	
	return rearSuspensionMeshRenderer;
}


ax::Node* createRearAxle(float width, float height, float depth) {
	// Create a cuboid for the rear axle
	ax::Mesh* rearAxleMesh = createCuboid(width, height, depth);
	
	ax::MeshRenderer* rearAxleMeshRenderer = ax::MeshRenderer::create();
	
	rearAxleMeshRenderer->addMesh(rearAxleMesh);

	// Set material, texture, and other properties as needed
	rearAxleMeshRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	rearAxleMeshRenderer->setTexture("gray.jpg");
	
	return rearAxleMeshRenderer;
}

ax::Node* createTransmission(float width, float height, float depth) {

	ax::Mesh* transmissionMeshUpper = createCuboid(width, height / 2.0f, depth / 2.0f);
	ax::Mesh* transmissionMeshLower = createCuboid(width, height / 2.0f, depth / 2.0f);
	ax::Mesh* transmissionMeshLeft = createCuboid(width, height / 2.0f, depth / 2.0f);
	ax::Mesh* transmissionMeshRight = createCuboid(width, height / 2.0f, depth / 2.0f);

	// Create a node to hold the transmission meshes
	ax::Node* transmissionMeshNode = ax::Node::create();
	
	// Create MeshRenderers for each side and set material and texture properties
	ax::MeshRenderer* transmissionMeshUpperRenderer = ax::MeshRenderer::create();
	transmissionMeshUpperRenderer->addMesh(transmissionMeshUpper);
	transmissionMeshUpperRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	transmissionMeshUpperRenderer->setTexture("gold.jpg");
	
	ax::MeshRenderer* transmissionMeshLowerRenderer = ax::MeshRenderer::create();
	transmissionMeshLowerRenderer->addMesh(transmissionMeshLower);
	transmissionMeshLowerRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	transmissionMeshLowerRenderer->setTexture("gold.jpg");
	
	ax::MeshRenderer* transmissionMeshLeftRenderer = ax::MeshRenderer::create();
	transmissionMeshLeftRenderer->addMesh(transmissionMeshLeft);
	transmissionMeshLeftRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	transmissionMeshLeftRenderer->setTexture("gold.jpg");
	
	ax::MeshRenderer* transmissionMeshRightRenderer = ax::MeshRenderer::create();
	transmissionMeshRightRenderer->addMesh(transmissionMeshRight);
	transmissionMeshRightRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
	transmissionMeshRightRenderer->setTexture("gold.jpg");
	
	transmissionMeshUpperRenderer->setPosition3D(ax::Vec3(0.0f, height, 0.0f));
	transmissionMeshLowerRenderer->setPosition3D(ax::Vec3(0.0f, -height, 0.0f));
	transmissionMeshLeftRenderer->setPosition3D(ax::Vec3(0.0f, 0.0f, -depth));
	transmissionMeshRightRenderer->setPosition3D(ax::Vec3(0.0f, 0.0f, depth));

	// Attach the MeshRenderers to the transmission node
	transmissionMeshNode->addChild(transmissionMeshUpperRenderer);
	transmissionMeshNode->addChild(transmissionMeshLowerRenderer);
	transmissionMeshNode->addChild(transmissionMeshLeftRenderer);
	transmissionMeshNode->addChild(transmissionMeshRightRenderer);
	
	return transmissionMeshNode;
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

ax::Node* createCarWithWheels(float carDimension, float wheelRadius, float wheelWidth, std::vector<ax::Node*>& suspensionContainer) {
	// Create the car body, roof, and windows as separate meshes
	ax::Node* carBody = createCarBody(carDimension);
	//ax::Node* roof = createCarRoof(carDimension, 1); // You may need to define roofHeight
	//ax::Node* windows = createCarWindows(carDimension, 0.25f);
	
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
	frontLeftWheel->setPosition3D(ax::Vec3(-carDimension + wheelRadius * 2.0f, -carDimension / 2, -carDimension / 2 - wheelRadius));
	frontRightWheel->setPosition3D(ax::Vec3(carDimension - wheelRadius * 2.0f, -carDimension / 2, carDimension / 2 + wheelRadius));
	rearLeftWheel->setPosition3D(ax::Vec3(-carDimension + wheelRadius * 2.0f, -carDimension / 2, carDimension / 2 + wheelRadius));
	rearRightWheel->setPosition3D(ax::Vec3(carDimension - wheelRadius * 2.0f, -carDimension / 2, -carDimension / 2 - wheelRadius));
	
	// Create rear suspension, rear axle, and transmission
	
	ax::Node* rearSuspension = createRearSuspension(0.0825f, 0.0825f, carDimension / 2.0f + wheelWidth / 2.0f);
	ax::Node* rearAxle = createRearAxle(1.0f, 0.1f, 0.1f);
	ax::Node* transmission = createTransmission(0.18f, 0.25f, 0.2f);
	
	float suspensionOffset = 0.2f;
	// Position the components relative to the car body
	rearSuspension->setPosition3D(ax::Vec3(-carDimension +  suspensionOffset, -carDimension / 2, 0.0f));
	rearAxle->setPosition3D(ax::Vec3(0.0f, -carDimension / 2, 0));
	transmission->setPosition3D(ax::Vec3(-carDimension +  suspensionOffset, -carDimension / 2, 0.0f));
	
	// Create a container node for the entire car (body, roof, windows, wheels, and components)
	ax::Node* carNode = ax::Node::create();
	carNode->addChild(carBody);
	carNode->addChild(frontLeftWheel);
	carNode->addChild(frontRightWheel);
	carNode->addChild(rearLeftWheel);
	carNode->addChild(rearRightWheel);
	carNode->addChild(rearSuspension);
	carNode->addChild(rearAxle);
	carNode->addChild(transmission);

	suspensionContainer.push_back(rearRightWheel);
	suspensionContainer.push_back(frontRightWheel);
	suspensionContainer.push_back(rearLeftWheel);
	suspensionContainer.push_back(frontLeftWheel);

	suspensionContainer.push_back(rearSuspension);

	// Attach roof and windows to the car body
	//carBody->addChild(roof);
	//carBody->addChild(windows);
	
	return carNode;
}
