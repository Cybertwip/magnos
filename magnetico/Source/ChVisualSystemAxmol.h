// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban, Alessandro Tasora
// =============================================================================

#ifndef CH_VISUAL_SYSTEM_IRRLICHT_H
#define CH_VISUAL_SYSTEM_IRRLICHT_H

#include <string>
#include <unordered_set>

#include <axmol.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChModelFileShape.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChLineShape.h"

//#include "chrono_irrlicht/ChApiIrr.h"
//#include "chrono_irrlicht/ChIrrNodeModel.h"
//#include "chrono_irrlicht/ChIrrCamera.h"
//#include "chrono_irrlicht/ChIrrEffects.h"
//#include "chrono_irrlicht/ChIrrGUI.h"

namespace chrono {
namespace axmol {

/// @addtogroup irrlicht_module
/// @{

/// Irrlicht-based Chrono run-time visualization system.
class ChVisualSystemAxmol : virtual public ChVisualSystem {
  public:
    ChVisualSystemAxmol();

    /// Auto-initialized run-time visualization system, with default settings.
	ChVisualSystemAxmol(ChSystem* sys, const ChVector<>& camera_pos = ChVector<>(2, 2, 2), const ChVector<>& camera_targ = ChVector<>(0, 0, 0));

    virtual ~ChVisualSystemAxmol();
	
	void Initialize(ax::Node* parent);

    /// Attach another Chrono system to the run-time visualization system.
    /// Currently only the first associated Chrono system is rendered. 
    virtual void AttachSystem(ChSystem* sys) override;

    /// Add a camera in an Irrlicht 3D scene.
    /// The camera rotation/pan is controlled by mouse left and right buttons, the zoom is controlled by mouse wheel or
    /// rmb+lmb+mouse, the position can be changed also with keyboard up/down/left/right arrows, the height can be
    /// changed with keyboard 'PgUp' and 'PgDn' keys. Optional parameters are position and target.
    /// Has no effect, unles called after Initialize().
    virtual int AddCamera(const ChVector<>& pos, ChVector<> targ = VNULL) override;

    /// Add a grid with specified parameters in the x-y plane of the given frame.
    virtual void AddGrid(double x_step,                           ///< grid cell size in X direction
                         double y_step,                           ///< grid cell size in Y direction
                         int nx,                                  ///< number of cells in X direction
                         int ny,                                  ///< number of cells in Y direction
                         ChCoordsys<> pos = CSYSNORM,             ///< grid reference frame
                         ChColor col = ChColor(0.1f, 0.1f, 0.1f)  ///< grid line color
                         ) override;

    /// Set the location of the specified camera.
    virtual void SetCameraPosition(int id, const ChVector<>& pos) override;

    /// Set the target (look-at) point of the specified camera.
    virtual void SetCameraTarget(int id, const ChVector<>& target) override;

    /// Set the location of the current (active) camera.
    virtual void SetCameraPosition(const ChVector<>& pos) override;

    /// Set the target (look-at) point of the current (active) camera.
    virtual void SetCameraTarget(const ChVector<>& target) override;

    /// Enable modal analysis visualization (default: false).
    /// If true, visualize an oscillatory motion of the n-th mode (only if some ChModalAssembly is found).
    /// Otherwise, visualize the dynamic evolution of the associated system.
    virtual void EnableModalAnalysis(bool val) override;

    /// Set the mode to be shown (only if some ChModalAssembly is found).
    virtual void SetModalModeNumber(int val) override;

    /// Set the amplitude of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalAmplitude(double val) override;

    /// Set the speed of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalSpeed(double val) override;

    /// Set the maximum number of modes selectable (only if some ChModalAssembly is found).
    virtual void SetModalModesMax(int maxModes) override;

    /// Process all visual assets in the associated ChSystem.
    /// This function is called by default by Initialize(), but can also be called later if further modifications to
    /// visualization assets occur.
    virtual void BindAll() override;

    /// Process the visual assets for the spcified physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its visual model
    /// occur after the call to Initialize().
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item) override;

    /// Add a visual model not associated with a physical item.
    /// Return a model ID which can be used later to modify the position of this visual model.
    virtual int AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) override;

    /// Add a visual model not associated with a physical item.
    /// This version constructs a visual model consisting of the single specified shape.
    /// Return an ID which can be used later to modify the position of this visual model.
    virtual int AddVisualModel(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) override;

    /// Update the position of the specified visualization-only model.
    virtual void UpdateVisualModel(int id, const ChFrame<>& frame) override;

    /// Run the Irrlicht device.
    /// Returns `false` if the device wants to be deleted.
    virtual bool Run() override;

    // Terminate the Irrlicht visualization.
    virtual void Quit() override;

    /// Perform any necessary operations at the beginning of each rendering frame.
    virtual void BeginScene() override;

    /// Draw all 3D shapes and GUI elements at the current frame.
    /// This function is typically called inside a loop such as
    /// <pre>
    ///    while(vis->Run()) {...}
    /// </pre>
    virtual void Render() override;

    /// Render the specified reference frame.
    virtual void RenderFrame(const ChFrame<>& frame, double axis_length = 1) override;

    /// Render COG frames for all bodies in the system.
    virtual void RenderCOGFrames(double axis_length = 1) override;

    /// End the scene draw at the end of each animation frame.
    virtual void EndScene() override;

    /// Create a snapshot of the last rendered frame and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) override;


  private:
    /// Perform necessary setup operations at the beginning of a time step.
    virtual void OnSetup(ChSystem* sys) override;

    /// Perform necessary update operations at the end of a time step.
    virtual void OnUpdate(ChSystem* sys) override;

    /// Remove all visualization objects from this visualization system.
    virtual void OnClear(ChSystem* sys) override;
	
private:
	void CreateAxmolNodes(const ChAssembly* assembly, std::unordered_set<const ChAssembly*>& trace);
	
	void CreateAxmolNode(std::shared_ptr<ChPhysicsItem> item);

	void PopulateAxmolNode(ax::Node* node,
						   std::shared_ptr<ChVisualModel> model,
						   const ChFrame<>& parent_frame);
	
	std::unordered_map<ChPhysicsItem*, ax::Node*> m_nodes;  ///< scene nodes for physics items
	std::vector<ax::Node*> m_vis_nodes;                    ///< scene nodes for vis-only models

};

/// @} irrlicht_module

}  // namespace irrlicht
}  // namespace chrono

#endif
