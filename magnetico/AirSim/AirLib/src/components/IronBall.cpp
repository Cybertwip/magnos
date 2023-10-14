#include "components/IronBall.hpp"

IronBall::IronBall(float radius) : MagneticBall(radius) {
    
}

float IronBall::get_density() const {
    return density;
}

float IronBall::get_relative_permeability() const {
    return relative_permeability;
}

float IronBall::get_magnetization() const {
    // Return saturation magnetization for Iron
    return M_s;
}
