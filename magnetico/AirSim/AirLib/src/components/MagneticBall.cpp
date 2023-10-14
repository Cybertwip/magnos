#include "components/MagneticBall.hpp"

MagneticBall::MagneticBall(float r) {
    this->radius = r;
}

MagneticBall::~MagneticBall() {}

float MagneticBall::get_magnetization() const {
    // Default magnetization is 0 for generic MagneticBall
    return 0.0f;
}

float MagneticBall::calculate_inertia() const {
    return (2.0f/5.0f) * calculate_mass() * std::pow(radius, 2);
}

float MagneticBall::getForceDueToMagneticField(const msr::airlib::Vector3r& B, const msr::airlib::Vector3r& gradB) const {
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
    
