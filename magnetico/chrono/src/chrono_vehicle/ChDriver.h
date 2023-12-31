// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle driver. A driver system must be able to report the
// current values of the inputs (throttle, steering, braking).
//
// =============================================================================

#ifndef CH_DRIVER_H
#define CH_DRIVER_H

#include <string>

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Base class for a vehicle driver system.
/// A driver system must be able to report the current values of the inputs
/// (throttle, steering, braking). A concrete driver class must set the member
/// variables m_throttle, m_steering, and m_braking.
class CH_VEHICLE_API ChDriver {
  public:
    /// Construct a driver subsystem associated with the given vehicle.
    ChDriver(ChVehicle& vehicle);

    virtual ~ChDriver() {}

    /// Get the driver throttle input (in the range [0,1]).
    double GetThrottle() const { return m_throttle; }

    /// Get the driver steering input (in the range [-1,+1]).
    double GetSteering() const { return m_steering; }

    /// Get the driver braking input (in the range [0,1]).
    double GetBraking() const { return m_braking; }

    /// Get the driver clutch input (in the range [0,1]).
    double GetClutch() const { return m_clutch; }

    /// Get all current inputs at once.
    DriverInputs GetInputs() const;

    /// Initialize this driver system.
    virtual void Initialize() {}

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time) {}

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step) {}

    /// Initialize output file for recording driver inputs.
    bool LogInit(const std::string& filename);

    /// Record the current driver inputs to the log file.
    bool Log(double time);

    /// Overwrite the value for the driver steering input (input is clamped in [-1,+1]).
    void SetSteering(double steering);

    /// Overwrite the value for the driver throttle input (input is clamped in [0,+1]).
    void SetThrottle(double throttle);

    /// Overwrite the value for the driver braking input (input is clamped in [0,+1]).
    void SetBraking(double braking);

    /// Overwrite the value for the clutch braking input (input is clamped in [0,+1]).
    void SetClutch(double clutch);

  protected:
    ChVehicle& m_vehicle;  ///< reference to associated vehicle
    double m_throttle;     ///< current value of throttle input
    double m_steering;     ///< current value of steering input
    double m_braking;      ///< current value of braking input
    double m_clutch;       ///< current value of clutch input

  private:
    std::string m_log_filename;  ///< name of output file for recording driver inputs
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif