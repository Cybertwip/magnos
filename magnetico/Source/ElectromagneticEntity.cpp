#include "ElectromagneticEntity.h"

ax::Mesh* ElectromagneticEntity::createCube(float dimension) {
		
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

