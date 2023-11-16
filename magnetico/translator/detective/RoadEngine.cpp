#include "RoadEngine.h"
#include "DepthEngine.h"

RoadEngine::RoadEngine()
{
	cfg.img_w = 640;
	cfg.img_h = 384;
	
	outputDataBuffer = std::make_unique<Image>(cfg.img_w, cfg.img_h, 3, std::vector<uint8_t>(cfg.img_w * cfg.img_h * 3));

}

const double kThresholdNormal = 0.1; // The angle threshold for the same plane
const double kRandomColorScale = 255.0;


std::tuple<Image> RoadEngine::detectLanes(ImagePointCloud& cloud){
//	
//	std::vector<uint8_t> rgbData;
//
//	std::vector<std::vector<int>> segments;
//	std::vector<std::vector<uint8_t>> segmentColors;
//	// Segment the point cloud based on the normal vectors
//	for (int i = 0; i < cloud.vertices.size(); ++i) {
//		bool assigned = false;
//		for (int j = 0; j < segments.size(); ++j) {
//			// Check if the point is part of the same plane based on the normal vector
//			double dotProduct = cloud.vertices[i].dot(cloud.vertices[segments[j][0]]);
//			double length1 = cloud.vertices[i].norm();
//			double length2 = cloud.vertices[segments[j][0]].norm();
//			double angle = std::acos(dotProduct / (length1 * length2));
//			
//			if (angle < kThresholdNormal) {
//				segments[j].push_back(i);
//				assigned = true;
//				break;
//			}
//		}
//		
//		if (!assigned) {
//			// Create a new segment for the point
//			segments.push_back({ i });
//			segmentColors.push_back({
//				static_cast<uint8_t>(std::rand() / kRandomColorScale),
//				static_cast<uint8_t>(std::rand() / kRandomColorScale),
//				static_cast<uint8_t>(std::rand() / kRandomColorScale)
//			});
//		}
//	}
//	
//	// Assign colors based on segments
//	rgbData.resize(3 * cloud.vertices.size(), 0);
//	
//	for (size_t i = 0; i < segments.size(); ++i) {
//		for (size_t j = 0; j < segments[i].size(); ++j) {
//			int index = segments[i][j];
//			int colorIndex = 3 * index;
//			rgbData[colorIndex] = segmentColors[i][0];
//			rgbData[colorIndex + 1] = segmentColors[i][1];
//			rgbData[colorIndex + 2] = segmentColors[i][2];
//		}
//	}
//
//	memset(outputDataBuffer->data.data(), 255, outputDataBuffer->data.size());
//	
//	memcpy(outputDataBuffer->data.data(), rgbData.data(), rgbData.size());
	
	return std::make_tuple(*outputDataBuffer);
}
