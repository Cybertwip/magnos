#include "MagneticBall.h"

MagneticBall::MagneticBall(float r) : ax::MeshRenderer() {
    this->radius = r;
    
    float sphereRadius = this->radius;
    unsigned int sphereRings = 20;
    unsigned int sphereSectors = 20;
    
    ax::Mesh* sphereMesh = createSphere(sphereRadius, sphereRings, sphereSectors);

    sphereMesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
    
    this->addMesh(sphereMesh);
    this->setTexture("gray.jpg");
    setPosition3D(ax::Vec3(0, 0, 0));
}

MagneticBall::~MagneticBall() {}

float MagneticBall::get_magnetization() const {
    // Default magnetization is 0 for generic MagneticBall
    return 0.0f;
}

float MagneticBall::calculate_inertia() const {
    return (2.0f/5.0f) * calculate_mass() * std::pow(radius, 2);
}


float MagneticBall::getForceDueToMagneticField(const ax::Vec3& B, const ax::Vec3& gradB) const {
    float mu_0 = 4.0f * M_PI * std::pow(10.0f, -7);
    float chi = get_relative_permeability() - 1.0f;
    float V = calculate_volume();

    return mu_0 * V * chi * B.dot(gradB);
}


float MagneticBall::calculate_volume() const {
    return (4.0f/3.0f) * M_PI * std::pow(radius, 3);
}


float MagneticBall::calculate_mass() const {
    return get_density() * calculate_volume();
}

float MagneticBall::calculate_permeability() const {
    return get_relative_permeability() * 4.0f * M_PI * 1e-7f;  // Tm/A
}
    
ax::Mesh* MagneticBall::createSphere(float radius, unsigned int rings, unsigned int sectors) {
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<float> texs;
    std::vector<unsigned short> indices;
    
    float const R = 1.0f / (float)(rings - 1);
    float const S = 1.0f / (float)(sectors - 1);
    
    for (unsigned int r = 0; r < rings; ++r) {
        for (unsigned int s = 0; s < sectors; ++s) {
            float const y = sin(-M_PI_2 + M_PI * r * R);
            float const x = cos(2 * M_PI * s * S) * sin(M_PI * r * R);
            float const z = sin(2 * M_PI * s * S) * sin(M_PI * r * R);
            
            texs.push_back(s * S);
            texs.push_back(r * R);
            
            positions.push_back(x * radius);
            positions.push_back(y * radius);
            positions.push_back(z * radius);
            
            normals.push_back(x);
            normals.push_back(y);
            normals.push_back(z);
        }
    }
    
    for (unsigned int r = 0; r < rings - 1; ++r) {
        for (unsigned int s = 0; s < sectors - 1; ++s) {
            // First triangle
            indices.push_back(r * sectors + s);
            indices.push_back((r + 1) * sectors + s);
            indices.push_back((r + 1) * sectors + (s + 1));
            
            // Second triangle
            indices.push_back(r * sectors + s);
            indices.push_back((r + 1) * sectors + (s + 1));
            indices.push_back(r * sectors + (s + 1));
        }
    }
    ax::IndexArray indexArray;
    for (auto index : indices) {
        indexArray.emplace_back(index);
    }
    
    return ax::Mesh::create(positions, normals, texs, indexArray);
}
