// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Simple engine model for the mrole vehicle based on torque-speed engine maps
//
// =============================================================================

#ifndef MROLE_ENGINE_SIMPLEMAP_H
#define MROLE_ENGINE_SIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// mrole simple speed-torque engine map subsystem.
class CH_MODELS_API mrole_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    mrole_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunction_Recorder::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) override;
};

/// @} vehicle_models_mrole

}  // end namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
