#pragma once

#include <axmol.h>

ax::Mesh* createCube(float dimension);
ax::Mesh* createTorus(float majorRadius, float minorRadius, int majorSegments, int minorSegments);
ax::Mesh* createFlatDisk(float radius, float thickness, int majorSegments, int minorSegments);

ax::Node* createCarWithWheels(float carDimension, float wheelRadius, float wheelWidth);

