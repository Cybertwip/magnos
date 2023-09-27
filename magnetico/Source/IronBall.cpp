#include "IronBall.h"

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

IronBall* IronBall::create(float radius) {
    IronBall* ball = new IronBall(radius);
    if (ball && ball->init()) {
        ball->autorelease();
        return ball;
    }
    delete ball;
    return nullptr;
}

