#if BUILD_COMPONENT_INSPECTOR
#include <codecvt>
#include <locale>

#include "chrono/utils/ChProfiler.h"
#include "chrono/core/ChMathematics.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChModelFileShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChBarrelShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#ifdef CHRONO_MODAL
    #include "chrono_modal/ChModalAssembly.h"
#endif

// @TODO integrate to the engine
//#include "chrono_irrlicht/ChVisualSystemAxmol.h"
//#include "chrono_irrlicht/ChIrrTools.h"
//#include "chrono_irrlicht/ChIrrMeshTools.h"
//#include "chrono_irrlicht/ChIrrSkyBoxSceneNode.h"

#include "ChVisualSystemAxmol.h"
#include "Utils3d.h"

namespace chrono {
namespace axmol {


void PopulateWithAxmolMaterial(ax::MeshRenderer* renderer, std::shared_ptr<ChVisualMaterial> mat) {
	ax::Material* material = ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::DIFFUSE_NOTEX, false);
	
	renderer->setMaterial(material);

//	float ns_val = mat->GetSpecularExponent();  // in [0, 1000]
//	irr_mat.Shininess = ns_val * 0.128f;
	
//	auto scale = mat->GetTextureScale();
	
	auto kd_texture_name = mat->GetKdTexture();
	if (!kd_texture_name.empty()) {
		renderer->setTexture(kd_texture_name);
		renderer->setColor(ax::Color3B::WHITE);
	} else {
		renderer->setTexture("white.jpg");
	}
	
	renderer->setColor(ax::Color3B(
								   mat->GetDiffuseColor().G * 255.0f,
								   mat->GetDiffuseColor().R * 255.0f,
								   mat->GetDiffuseColor().B * 255.0f));

	//renderer->setOpacity(mat->GetOpacity() * 255.0f);
}

static void SetVisualMaterial(
							  ax::MeshRenderer* renderer,
							  ax::Mesh* mesh,
							  std::shared_ptr<ChVisualMaterial> material) {
	if (material == nullptr) {
		// Use default material
		mesh->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::DIFFUSE, false));
		renderer->setMaterial(mesh->getMaterial());
		renderer->setTexture("gray.jpg");
		renderer->setColor(ax::Color3B::WHITE);
	} else {
		PopulateWithAxmolMaterial(renderer, material);
		mesh->setMaterial(renderer->getMaterial());
	}
}

using namespace axmol;

//static std::shared_ptr<video::SMaterial> default_material;

ChVisualSystemAxmol::ChVisualSystemAxmol()
{
    
}

ChVisualSystemAxmol::ChVisualSystemAxmol(ChSystem* sys, const ChVector<>& camera_pos, const ChVector<>& camera_targ)
    : ChVisualSystemAxmol()
{
    AttachSystem(sys);
    AddCamera(camera_pos, camera_targ);
}

ChVisualSystemAxmol::~ChVisualSystemAxmol() {
}

void ChVisualSystemAxmol::Initialize(ax::Node* parent){
	auto container = ax::Node::create();
	for(auto node : m_nodes){
		container->addChild(node.second);
	}
	
	container->setRotation3D(ax::Vec3(0, 0, 0));
	parent->addChild(container);
}


// -----------------------------------------------------------------------------


void ChVisualSystemAxmol::AttachSystem(ChSystem* sys) {
    ChVisualSystem::AttachSystem(sys);

	BindAll();
}

// -----------------------------------------------------------------------------

bool ChVisualSystemAxmol::Run() {
}

void ChVisualSystemAxmol::Quit() {
}

void ChVisualSystemAxmol::OnSetup(ChSystem* sys) {
//    PurgeIrrNodes();
}

void ChVisualSystemAxmol::OnUpdate(ChSystem* sys) {
    for (auto& node : m_nodes) {
		ChFrame<> shape_frame = node.first->GetVisualModelFrame();
		
		ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
									 shape_frame.GetPos().y(),
									 shape_frame.GetPos().z());
		
		ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
												 shape_frame.GetRot().e2(),
												 shape_frame.GetRot().e3(),
												 shape_frame.GetRot().e0());
		
		
		node.second->setPosition3D(position);
		node.second->setRotationQuat(rotation);
    }
}

void ChVisualSystemAxmol::OnClear(ChSystem* sys) {
//    for (auto& node : m_nodes) {
//        node.second->removeAll();
//        node.second->remove();
//    }
//    m_nodes.clear();
}

//void ChVisualSystemAxmol::PurgeIrrNodes() {
//    // Remove Irrlicht nodes associated with a deleted physics item
//    std::vector<ChPhysicsItem*> items_to_remove;
//    for (auto& node : m_nodes) {
//        if (node.second->GetPhysicsItem().expired()) {
//            node.second->removeAll();
//            node.second->remove();
//            items_to_remove.emplace_back(node.first);
//        }
//    }
//
//    //// RADU TODO - what if the visual model of the associated node was modified?!?!
//    ////   We may now have Irrlicht scene nodes associated with visual shapes that no longer exist!
//
//    for (auto&& item : items_to_remove)
//        m_nodes.erase(item);
//}

// -----------------------------------------------------------------------------

int ChVisualSystemAxmol::AddCamera(const ChVector<>& pos, ChVector<> targ) {
//    if (!m_device)
//        return -1;
//
//    // create and init camera
//    auto camera = chrono_types::make_shared<RTSCamera>(m_device, GetSceneManager()->getRootSceneNode(),
//                                                       GetSceneManager(), -1, -160.0f, 1.0f, 0.003f);
//    // camera->bindTargetAndRotation(true);
//    if (!m_yup)
//        camera->setZUp();
//    camera->setPosition(core::vector3dfCH(pos));
//    camera->setTarget(core::vector3dfCH(targ));
//
//    camera->setNearValue(0.1f);
//    camera->setMinZoom(0.6f);
//
//    m_cameras.push_back(camera);
//    return (int)m_cameras.size() - 1;
}

void ChVisualSystemAxmol::AddGrid(double x_step, double y_step, int nx, int ny, ChCoordsys<> pos, ChColor col) {
//    m_grids.push_back({x_step, y_step, nx, ny, pos, col});
}

void ChVisualSystemAxmol::SetCameraPosition(int id, const ChVector<>& pos) {
//    m_cameras[id]->setPosition(core::vector3dfCH(pos));
}

void ChVisualSystemAxmol::SetCameraTarget(int id, const ChVector<>& target) {
//    m_cameras[id]->setTarget(core::vector3dfCH(target));
}

void ChVisualSystemAxmol::SetCameraPosition(const ChVector<>& pos) {
//    GetActiveCamera()->setPosition(core::vector3dfCH(pos));
}

void ChVisualSystemAxmol::SetCameraTarget(const ChVector<>& target) {
//    GetActiveCamera()->setTarget(core::vector3dfCH(target));
}


// -----------------------------------------------------------------------------

void ChVisualSystemAxmol::EnableModalAnalysis(bool val) {
}

void ChVisualSystemAxmol::SetModalModeNumber(int val) {
}

void ChVisualSystemAxmol::SetModalAmplitude(double val) {
}

void ChVisualSystemAxmol::SetModalSpeed(double val) {
}

void ChVisualSystemAxmol::SetModalModesMax(int maxModes) {
}

// Clean canvas at beginning of scene.

void ChVisualSystemAxmol::BeginScene() {
//    BeginScene(true, true, ChColor(0, 0, 0));
}

// Call this to end the scene draw at the end of each animation frame.
void ChVisualSystemAxmol::EndScene() {
}

void ChVisualSystemAxmol::Render() {
    
}

void ChVisualSystemAxmol::RenderFrame(const ChFrame<>& frame, double axis_length) {
//    const auto& loc = frame.GetPos();
//    const auto& u = frame.GetA().Get_A_Xaxis();
//    const auto& v = frame.GetA().Get_A_Yaxis();
//    const auto& w = frame.GetA().Get_A_Zaxis();
//    irrlicht::tools::drawSegment(this, loc, loc + u * axis_length, ChColor(1, 0, 0));
//    irrlicht::tools::drawSegment(this, loc, loc + v * axis_length, ChColor(0, 1, 0));
//    irrlicht::tools::drawSegment(this, loc, loc + w * axis_length, ChColor(0, 0, 1));
}

void ChVisualSystemAxmol::RenderCOGFrames(double axis_length) {
//    irrlicht::tools::drawAllCOGs(this, axis_length);
}

void ChVisualSystemAxmol::WriteImageToFile(const std::string& filename) {
//    video::IImage* image = GetVideoDriver()->createScreenShot();
//    if (image) {
//        GetVideoDriver()->writeImageToFile(image, filename.c_str());
//        image->drop();
//    }
}

// -----------------------------------------------------------------------------

void ChVisualSystemAxmol::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    if (m_systems.empty())
        return;

//    CreateIrrNode(item);
}

void ChVisualSystemAxmol::BindAll() {
    if (m_systems.empty())
        return;
//
//    PurgeIrrNodes();
//
    std::unordered_set<const ChAssembly*> trace;
    CreateAxmolNodes(&m_systems[0]->GetAssembly(), trace);
}

int ChVisualSystemAxmol::AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) {
	
	
	
//    // Create an Irrlicht scene node for a visualization-only model and populate it
//    auto node = chrono_types::make_shared<ChIrrNodeVisual>(m_container, GetSceneManager());
//    PopulateIrrNode(node.get(), model, frame);
//
//    core::matrix4CH irrMat(frame);
//    node->setPosition(irrMat.getTranslation());
//    node->setRotation(irrMat.getRotationDegrees());
//
//    // Cache the new node and return its ID
//    m_vis_nodes.push_back(node);
//    return (int)m_vis_nodes.size() - 1;
}

int ChVisualSystemAxmol::AddVisualModel(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    auto model = chrono_types::make_shared<ChVisualModel>();
    model->AddShape(shape);
    return AddVisualModel(model, frame);
}

void ChVisualSystemAxmol::UpdateVisualModel(int id, const ChFrame<>& frame) {
//    assert(id >= 0 && id < m_vis_nodes.size());
//
//    core::matrix4CH irrMat(frame);
//    m_vis_nodes[id]->setPosition(irrMat.getTranslation());
//    m_vis_nodes[id]->setRotation(irrMat.getRotationDegrees());
}
//
void ChVisualSystemAxmol::CreateAxmolNodes(const ChAssembly* assembly, std::unordered_set<const ChAssembly*>& trace) {
//    // Do nothing if the assembly was already processed
    if (!trace.insert(assembly).second)
        return;

    for (auto& body : assembly->Get_bodylist()) {
		CreateAxmolNode(body);
    }

    for (auto& link : assembly->Get_linklist()) {
		CreateAxmolNode(link);
    }

    for (auto& mesh : assembly->Get_meshlist()) {
		CreateAxmolNode(mesh);
    }

    for (auto& ph : assembly->Get_otherphysicslist()) {
        CreateAxmolNode(ph);

        // Recursively process sub-assemblies
        if (auto a = std::dynamic_pointer_cast<ChAssembly>(ph)) {
            CreateAxmolNodes(a.get(), trace);
        }
    }
//
//#ifdef CHRONO_MODAL
//    // Modal assemblies contain custom internal items that might be useful to visualize
//    if (auto assy_modal = dynamic_cast<const chrono::modal::ChModalAssembly*>(assembly)) {
//        for (auto body : assy_modal->Get_internal_bodylist()) {
//            CreateIrrNode(body);
//        }
//        for (auto& mesh : assy_modal->Get_internal_meshlist()) {
//            CreateIrrNode(mesh);
//        }
//        for (auto ph : assy_modal->Get_internal_otherphysicslist()) {
//            CreateIrrNode(ph);
//            // If the assembly holds another assemblies, also bind their contents.
//            if (auto a = std::dynamic_pointer_cast<ChAssembly>(ph)) {
//                CreateIrrNodes(a.get(), trace);
//            }
//        }
//        for (auto link : assy_modal->Get_internal_linklist()) {
//            CreateIrrNode(link);
//        }
//    }
//#endif
}
//
void ChVisualSystemAxmol::CreateAxmolNode(std::shared_ptr<ChPhysicsItem> item) {
    if (!item->GetVisualModel())
        return;

//    // Do nothing if an Irrlicht node already exists for this physics item
    if (m_nodes.find(item.get()) != m_nodes.end())
        return;
	
//
//    // Create a new ChIrrNodeModel and populate it
    auto node = ax::Node::create();
    bool ok = m_nodes.insert({item.get(), node}).second;
    assert(ok);

    ax::Node* fillnode = node;

//    // Recursively populate the ChIrrNodeModel with Irrlicht scene nodes for each visual shape.
//    // Begin with identity transform relative to the physics item.
    ChFrame<> shape_frame = item->GetVisualModelFrame();
	
	ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
								 shape_frame.GetPos().y(),
								 shape_frame.GetPos().z());
	
	ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
											 shape_frame.GetRot().e2(),
											 shape_frame.GetRot().e3(),
											 shape_frame.GetRot().e0());

	
	fillnode->setPosition3D(position);
	fillnode->setRotationQuat(rotation);

    PopulateAxmolNode(fillnode, item->GetVisualModel(), shape_frame);
}
//
void ChVisualSystemAxmol::PopulateAxmolNode(ax::Node* node,
                                             std::shared_ptr<ChVisualModel> model,
                                             const ChFrame<>& parent_frame) {
    for (const auto& shape_instance : model->GetShapes()) {
        auto& shape = shape_instance.first;
        auto& shape_frame = shape_instance.second;

        if (!shape->IsVisible())
            continue;

        if (auto obj = std::dynamic_pointer_cast<ChModelFileShape>(shape)) {
            
			auto renderer = ax::MeshRenderer::create(obj->GetFilename());
			node->addChild(renderer);

			ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
										 shape_frame.GetPos().y(),
										 shape_frame.GetPos().z());

			ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
													 shape_frame.GetRot().e2(),
													 shape_frame.GetRot().e3(),
													 shape_frame.GetRot().e0());

			renderer->setPosition3D(position);
			renderer->setRotationQuat(rotation);
			
			for(int i = 0; i<renderer->getMeshCount(); ++i){
				SetVisualMaterial(renderer, renderer->getMeshes().at(i), shape->GetMaterials().empty() ? nullptr : shape->GetMaterial(i));
			}

		} else if (auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {

			std::vector<float> positions;
			std::vector<float> normals;
			ax::IndexArray indexArray(ax::backend::IndexFormat::U_INT);
			

			// Texture coordinates for the plane
			std::vector<float> texs;
			
			// Function to add a vertex to the map and return its index
			auto addVertex = [&](const ChVector<>& vertex) -> unsigned int {
				positions.push_back(static_cast<float>(vertex.x()));
				positions.push_back(static_cast<float>(vertex.y()));
				positions.push_back(static_cast<float>(vertex.z()));
				
				return static_cast<unsigned int>(positions.size() / 3);
			};
			
			
			unsigned int n_faces = trimesh->GetMesh()->getNumTriangles();
			
			// Assuming you have a function addVertex that adds a vertex to your positions vector
			// with coordinates and returns its index
			for (unsigned int it = 0; it < n_faces; it++) {
				auto triangle = trimesh->GetMesh()->getTriangle(it);
				
				// Triangle vertices
				const ChVector<>& p1 = triangle.p1;
				const ChVector<>& p2 = triangle.p2;
				const ChVector<>& p3 = triangle.p3;
				
				// Create a triangle using the triangle vertices
				addVertex(p1);
				addVertex(p2);
				addVertex(p3);
				
				// Add indices for the vertices in the triangle
				indexArray.emplace_back(static_cast<unsigned int>(it * 3));
				indexArray.emplace_back(static_cast<unsigned int>(it * 3 + 1));
				indexArray.emplace_back(static_cast<unsigned int>(it * 3 + 2));

				auto normal1 = trimesh->GetMesh()->getCoordsNormals()[it];

				auto normal2 = trimesh->GetMesh()->getCoordsNormals()[it + 1];

				auto normal3 = trimesh->GetMesh()->getCoordsNormals()[it + 2];

				// Modify this part to add normals to your normal data structure
				normals.push_back(normal1.x());
				normals.push_back(normal1.y());
				normals.push_back(normal1.z());

				
				normals.push_back(normal2.x());
				normals.push_back(normal2.y());
				normals.push_back(normal2.z());

				
				normals.push_back(normal3.x());
				normals.push_back(normal3.y());
				normals.push_back(normal3.z());

				// Access UV coordinates from the mesh, not from trimesh
				auto uv0 = trimesh->GetMesh()->getCoordsUV()[trimesh->GetMesh()->getIndicesUV()[it][0]];
				auto uv1 = trimesh->GetMesh()->getCoordsUV()[trimesh->GetMesh()->getIndicesUV()[it][1]];
				auto uv2 = trimesh->GetMesh()->getCoordsUV()[trimesh->GetMesh()->getIndicesUV()[it][2]];
				
				// Modify this part to add UV coordinates to your UV data structure
				texs.push_back(static_cast<float>(uv0.x()));
				texs.push_back(static_cast<float>(uv0.y()));
				texs.push_back(static_cast<float>(uv1.x()));
				texs.push_back(static_cast<float>(uv1.y()));
				texs.push_back(static_cast<float>(uv2.x()));
				texs.push_back(static_cast<float>(uv2.y()));
			}

			
			auto mesh = ax::Mesh::create(positions, normals, texs, indexArray);
			
			mesh->getMeshIndexData()->setPrimitiveType(ax::MeshCommand::PrimitiveType::TRIANGLE_STRIP);
			
			auto renderer = ax::MeshRenderer::create();
			
			renderer->addMesh(mesh);

			SetVisualMaterial(renderer, mesh, shape->GetMaterials().empty() ? nullptr : shape->GetMaterial(0));
			
			node->addChild(renderer);
			

			ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
										 shape_frame.GetPos().y(),
										 shape_frame.GetPos().z());
			
			
			ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
													 shape_frame.GetRot().e2(),
													 shape_frame.GetRot().e3(),
													 shape_frame.GetRot().e0());

			renderer->setPosition3D(position);
			renderer->setRotationQuat(rotation);
//			renderer->setCullFace(trimesh->IsBackfaceCull() ? ax::CullFaceSide::FRONT : ax::CullFaceSide::BACK);

//            SetVisualMaterial(mchildnode, shape);
//            mchildnode->setMaterialFlag(video::EMF_WIREFRAME, trimesh->IsWireframe());
//            mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trimesh->IsBackfaceCull());
        }
		else if (auto surf = std::dynamic_pointer_cast<ChSurfaceShape>(shape)) {
			bool ok = 0;

//            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
//            SMesh* newmesh = new SMesh;
//            newmesh->addMeshBuffer(buffer);
//            buffer->drop();
//
//            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(surf, node);
//            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
//            newmesh->drop();
//
//            mchildnode->setPosition(shape_m4.getTranslation());
//            mchildnode->setRotation(shape_m4.getRotationDegrees());
//
//            mproxynode->Update();  // force syncing of triangle positions & face indexes
//            mproxynode->drop();
//
//            SetVisualMaterial(mchildnode, shape);
//            mchildnode->setMaterialFlag(video::EMF_WIREFRAME, surf->IsWireframe());
//        } else if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
//            if (sphereMesh) {
//                ISceneNode* mproxynode = new ChIrrNodeShape(sphere, node);
//                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(sphereMesh, mproxynode);
//                mproxynode->drop();
//
//                double mradius = sphere->GetRadius();
//                mchildnode->setScale(core::vector3dfCH(ChVector<>(mradius, mradius, mradius)));
//                mchildnode->setPosition(shape_m4.getTranslation());
//                mchildnode->setRotation(shape_m4.getRotationDegrees());
//
//                SetVisualMaterial(mchildnode, sphere);
//                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
			bool ok  = false;
//            if (sphereMesh) {
//                ISceneNode* mproxynode = new ChIrrNodeShape(ellipsoid, node);
//                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(sphereMesh, mproxynode);
//                mproxynode->drop();
//
//                mchildnode->setScale(core::vector3dfCH(ellipsoid->GetSemiaxes()));
//                mchildnode->setPosition(shape_m4.getTranslation());
//                mchildnode->setRotation(shape_m4.getRotationDegrees());
//
//                SetVisualMaterial(mchildnode, ellipsoid);
//                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
//            }
        } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {

			double rad = cylinder->GetRadius();
			double height = cylinder->GetHeight();

			auto mesh = createCylinder(rad, height, 20);
			
			auto renderer = ax::MeshRenderer::create();
			
			renderer->addMesh(mesh);

			SetVisualMaterial(renderer, mesh, shape->GetMaterials().empty() ? nullptr : shape->GetMaterial(0));
			
			node->addChild(renderer);
			
			
			ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
										 shape_frame.GetPos().y(),
										 shape_frame.GetPos().z());
			
			
			
			ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
													 shape_frame.GetRot().e2(),
													 shape_frame.GetRot().e3(),
													 shape_frame.GetRot().e0());
			renderer->setPosition3D(position);
			renderer->setRotationQuat(rotation);
			
        } else if (auto capsule = std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
			
			double rad = capsule->GetRadius();
			double height = capsule->GetHeight();

			auto mesh = createCapsule(rad, height, 20, 20, 20);
			
			auto renderer = ax::MeshRenderer::create();
			
			renderer->addMesh(mesh);
			
			SetVisualMaterial(renderer, mesh, shape->GetMaterials().empty() ? nullptr : shape->GetMaterial(0));
			
			node->addChild(renderer);
			
			
			ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
										 shape_frame.GetPos().y(),
										 shape_frame.GetPos().z());
			
			
			
			ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
													 shape_frame.GetRot().e2(),
													 shape_frame.GetRot().e3(),
													 shape_frame.GetRot().e0());
			renderer->setPosition3D(position);
			renderer->setRotationQuat(rotation);
			
        } else if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
			
			auto mesh = createCuboid(box->GetHalflengths().x(), box->GetHalflengths().y(), box->GetHalflengths().z());
			
			auto renderer = ax::MeshRenderer::create();
			
			renderer->addMesh(mesh);

			SetVisualMaterial(renderer, mesh, shape->GetMaterials().empty() ? nullptr : shape->GetMaterial(0));
			
			node->addChild(renderer);
			
			
			ax::Vec3 position = ax::Vec3(shape_frame.GetPos().x(),
										 shape_frame.GetPos().y(),
										 shape_frame.GetPos().z());
			
			
			
			ax::Quaternion rotation = ax::Quaternion(shape_frame.GetRot().e1(),
													 shape_frame.GetRot().e2(),
													 shape_frame.GetRot().e3(),
													 shape_frame.GetRot().e0());
			renderer->setPosition3D(position);
			renderer->setRotationQuat(rotation);
			
        } else if (auto glyphs = std::dynamic_pointer_cast<ChGlyphs>(shape)) {
//            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
//            SMesh* newmesh = new SMesh;
//            newmesh->addMeshBuffer(buffer);
//            buffer->drop();
//
//            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(glyphs, node);
//            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
//            newmesh->drop();
//
//            mproxynode->Update();  // force syncing of triangle positions & face indexes
//            mproxynode->drop();
//
//            SetVisualMaterial(mchildnode, glyphs);
//
//            ////mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
//            ////mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
//        } else if (std::dynamic_pointer_cast<ChPathShape>(shape) || std::dynamic_pointer_cast<ChLineShape>(shape)) {
//            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(video::EVT_STANDARD, video::EIT_32BIT);
//            SMesh* newmesh = new SMesh;
//            newmesh->addMeshBuffer(buffer);
//            buffer->drop();
//
//            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(shape, node);
//            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
//            newmesh->drop();
//
//            mproxynode->Update();  // force syncing of triangle positions & face indexes
//            mproxynode->drop();
//
//            mchildnode->setPosition(shape_m4.getTranslation());
//            mchildnode->setRotation(shape_m4.getRotationDegrees());
//
//            SetVisualMaterial(mchildnode, shape);
//
//            ////mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
//            ////mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
        }
    }
}

}  // namespace irrlicht
}  // namespace chrono
#endif
