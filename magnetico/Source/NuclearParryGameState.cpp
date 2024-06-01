#include "NuclearParryGameState.h"

#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"
#include "Plane.h"
#include "Utils3d.h"

#include "ImGui/ImGuiPresenter.h"
#include "imgui/imgui_internal.h"


USING_NS_AX;

namespace{
Vec3 cross(const Vec3& a, const Vec3& b) {
	return Vec3(
				a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x
				);
}
}

NuclearParryGameState::NuclearParryGameState() {
	// Initialize particle states vector
	activeParticles.resize(initial_particles);
}

void NuclearParryGameState::initializeParticles() {
	// Define central activation point
	int center_x = grid_size / 2;
	int center_y = grid_size / 2;
	int center_z = grid_size / 2;
	
	// Randomly activate initial particles around the center
	for (int i = 0; i < initial_particles; ++i) {
		int x = center_x + rand() % activation_radius;
		int y = center_y + rand() % activation_radius;
		int z = center_z + rand() % activation_radius;
		if (x < grid_size && y < grid_size && z < grid_size) {
			// Set particle position
			activeParticles[i].position = ax::Vec3(x, y, z);
			// Set activation status randomly
			activeParticles[i].activated = rand() % 2 == 0; // Randomly true or false
		}
	}
}



bool NuclearParryGameState::init() {
	//////////////////////////////
	// 1. super init first
	if (!Node::init()) {
		return false;
	}
	
	// Initialize particles
	initializeParticles();
	// Loop through particles and add them to the scene
	for (const auto& particle : activeParticles) {
		if (particle.activated) { // Only add if activated
			// Create a sphere mesh for each particle
			ax::Mesh* mesh = createSphere(0.1f, 20, 20); // Adjust parameters as needed
			
			ax::MeshRenderer* renderer = ax::MeshRenderer::create();
			
			renderer->addMesh(mesh);
			
			// Set material, texture, and other properties as needed
			renderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
			renderer->setTexture("kitty.jpg");
			
			// Position the sphere at the particle's coordinates
			renderer->setPosition3D(particle.position); // Adjust position as needed
			
			particleRenderers.push_back(renderer);
			
			
			// Add the sphere to the scene
			this->addChild(renderer);
		}
	}

	return true;
}


void NuclearParryGameState::setup(ax::Camera* defaultCamera){
	_defaultCamera = defaultCamera;
	
	_defaultCamera->setNearPlane(0.01f);
	_defaultCamera->setFarPlane(10000);
	_defaultCamera->setFOV(90);
	_defaultCamera->setZoom(1);
	_defaultCamera->setPosition3D(Vec3(1.5f, 1.5f, -1.5f));
	_defaultCamera->setRotation3D(Vec3(0, 0, 0));
	
	_defaultCamera->lookAt(Vec3(0, 0, 0));
}

void NuclearParryGameState::onMouseMove(Event* event)
{
	EventMouse* e = static_cast<EventMouse*>(event);
	// Get the cursor delta since the last frame
	
	prevCursorX = cursorX;
	prevCursorY = cursorY;
	
	cursorX = e->getDelta().x;
	cursorY = e->getDelta().y;
	
	cursorDeltaX = cursorX - prevCursorX;
	cursorDeltaY = cursorY - prevCursorY;
}


void NuclearParryGameState::onKeyPressed(EventKeyboard::KeyCode code, Event* event) {
	switch (code) {
		case EventKeyboard::KeyCode::KEY_W:
			// Move camera forward
			moveForward = true;
			break;
		case EventKeyboard::KeyCode::KEY_A:
			// Move camera left
			moveLeft = true;
			break;
		case EventKeyboard::KeyCode::KEY_S:
			// Move camera backward
			moveBackward = true;
			break;
		case EventKeyboard::KeyCode::KEY_D:
			// Move camera right
			moveRight = true;
			break;
		case EventKeyboard::KeyCode::KEY_SPACE:
			react = true;
		default:
			break;
	}
}

void NuclearParryGameState::onKeyReleased(EventKeyboard::KeyCode code, Event* event) {
	switch (code) {
		case EventKeyboard::KeyCode::KEY_W:
			// Stop moving camera forward
			moveForward = false;
			break;
		case EventKeyboard::KeyCode::KEY_A:
			// Stop moving camera left
			moveLeft = false;
			break;
		case EventKeyboard::KeyCode::KEY_S:
			// Stop moving camera backward
			moveBackward = false;
			break;
		case EventKeyboard::KeyCode::KEY_D:
			// Stop moving camera right
			moveRight = false;
			break;
		case EventKeyboard::KeyCode::KEY_SPACE:
			react = false;
		default:
			break;
	}
}

void NuclearParryGameState::update(float delta) {
	// Calculate new camera rotation angles based on normalized cursor deltas
	horizontalAngle += cursorDeltaX * sensitivity;
	verticalAngle -= cursorDeltaY * sensitivity;
	
	// Calculate the new camera position relative to the current position
	Vec3 forward(cos(verticalAngle) * sin(horizontalAngle), sin(verticalAngle), cos(verticalAngle) * cos(horizontalAngle));
	Vec3 right = cross(Vec3::UNIT_Y, forward);
	Vec3 up = cross(forward, right);

	// WASD movement
	Vec3 movement(0.0f, 0.0f, 0.0f);
	if (moveForward) movement += forward;
	if (moveBackward) movement -= forward;
	if (moveLeft) movement += right;
	if (moveRight) movement -= right;
	
	// Normalize movement vector to maintain constant speed
	if (movement.lengthSquared() > 0.0f) movement.normalize();
	
	// Update camera position
	Vec3 newPosition = _defaultCamera->getPosition3D() + (movement * delta * 10);
	_defaultCamera->setPosition3D(newPosition);
	
	// Calculate the new look-at point based on mouse movement
	Vec3 lookAtPoint = newPosition + forward;
	
	// Set the camera's new look-at point
	_defaultCamera->lookAt(lookAtPoint);
	
	cursorDeltaX = 0;
	cursorDeltaY = 0;
	
	if (react) {
		// Propagate particles outward to form a mushroom cloud
		std::vector<Particle> newParticles;
		for (auto& particle : activeParticles) {
			int x = particle.position.x;
			int y = particle.position.y;
			int z = particle.position.z;
			if (z < grid_size - 1) {
				ax::Vec3 newPos(x, y, z + 1);
				if (z > 0) {
					// Spread outward
					if (x > 0) newParticles.push_back({ax::Vec3(x - 1, y, z), rand() % 2 == 0});
					if (x < grid_size - 1) newParticles.push_back({ax::Vec3(x + 1, y, z), rand() % 2 == 0});
					if (y > 0) newParticles.push_back({ax::Vec3(x, y - 1, z), rand() % 2 == 0});
					if (y < grid_size - 1) newParticles.push_back({ax::Vec3(x, y + 1, z), rand() % 2 == 0});
				}
				// Update the particle's position
				particle.position = newPos;
			}
		}
		
		// Create new particle renderers for activated particles
		for (const auto& particle : newParticles) {
			if (particle.activated) {
				// Create a sphere mesh for each activated particle
				ax::Mesh* mesh = createSphere(0.1f, 20, 20); // Adjust parameters as needed
				ax::MeshRenderer* renderer = ax::MeshRenderer::create();
				renderer->addMesh(mesh);
				renderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
				renderer->setTexture("kitty.jpg");
				renderer->setPosition3D(particle.position); // Adjust position as needed
				particleRenderers.push_back(renderer);
				this->addChild(renderer); // Add the sphere to the scene
			}
		}
		
		// Add the new particles to the activeParticles vector
		activeParticles.insert(activeParticles.end(), newParticles.begin(), newParticles.end());
	}
}


void NuclearParryGameState::renderUI() {
	ImGui::SetNextWindowPos(ImVec2(120, 60), ImGuiCond_FirstUseEver);
	
	ImGui::Begin("Nuclear Parry");
	
	ImGui::End();
}
