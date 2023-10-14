#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "physics/Kinematics.hpp"
#include "CarPawn.h"

class EVEngine;

class CarPawnApi
{
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    CarPawnApi(ACarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
               msr::airlib::CarApiBase* vehicle_api);

    void updateMovement(const msr::airlib::CarApiBase::CarControls& controls);

	msr::airlib::CarApiBase::CarState getCarState() const;

	std::shared_ptr<EVEngine> getEngine() const;

    void reset();
    void update();

    virtual ~CarPawnApi();

private:
    UChaosWheeledVehicleMovementComponent* movement_;
    msr::airlib::CarApiBase::CarControls last_controls_;
    ACarPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::CarApiBase* vehicle_api_;
	
	std::shared_ptr<EVEngine> engine_;
};
