#include "StbUtils.h"
#include "OnnxEngine.h"


int main(int argc, char** argv) {
	
	
	
}

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/filters/extract_indices.h>
//
//#define RED_MULTIPLIER 0.299
//#define GREEN_MULTIPLIER 0.587
//#define BLUE_MULTIPLIER 0.114
//#define MAX_COLOR_INTENSITY 255
//
//struct Intr {
//	int width, height;
//	double fx, fy, cx, cy, scale_factor;
//};
//const Intr DEFAULT_CAM_PARAMS = {1280, 720, 640, 640, 640.0, 360.0, (1.0 / 5000.0)};
//
//
//int main()
//{
//	
//	// Initialization...
//	DepthEngine engine(3);
//	
//	std::string input_path = std::string{MODELS_PATH} + "input.png";
//	std::string depth_path = std::string{MODELS_PATH} + "depth.png";
//	
//	auto inputImage = load_image_data(input_path);
//	auto [depthImage] =  engine.detectDepth(inputImage);
//	
//	saveImage(depthImage, depth_path);
//	
//	// Create PCL point cloud from depth and color images
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	coloredCloud->width = depthImage.width;
//	coloredCloud->height = depthImage.height;
//	coloredCloud->is_dense = true;  // Assuming there won't be NaN values
//	
//	for (int y = 0; y < depthImage.width; ++y) {
//		for (int x = 0; x < depthImage.height; ++x) {
//			pcl::PointXYZRGB point;
//			
//			float z = static_cast<float>(depthImage[x][y]); // Depth value for pixel (j, k)
//			point.x = (x - kCx) * z / kFx; // Use the depth value to multiply the x coordinate
//			point.y = (y - kCy) * z / kFy; // Use the depth value to multiply the y coordinate
//			
//			point.z = z;
//
//			point.r = inputImage[x][y];
//			point.g = inputImage[x][y + 1];
//			point.b = inputImage[x][y + 2];
////
//			coloredCloud->push_back(point);
//		}
//	}
//
//	// Apply RANSAC to detect planes
//	// Create a SampleConsensusModelPlane
//	typename pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(coloredCloud));
//	
//	// Create a RandomSampleConsensus object
//	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
//	ransac.setDistanceThreshold(100);
//	ransac.setMaxIterations(1000);
//	
//	// Compute the model
//	ransac.computeModel();
//	
//	// Get the inliers and coefficients
//	std::shared_ptr<std::vector<int>> inliers = std::make_shared<std::vector<int>>();
//	ransac.getInliers(*inliers);
//	Eigen::VectorXf obtained_model_coefficients;
//	ransac.getModelCoefficients(obtained_model_coefficients);
//	
//	// Extract inliers (points belonging to the plane)
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//	extract.setInputCloud(coloredCloud);
//	extract.setIndices(inliers);
//	extract.setNegative(false); // Extract the inliers (plane)
//	extract.filter(*cloud_plane);
//
//	
//	
//	// Use normals to determine surface type
////	for (std::size_t i = 0; i < cloud_plane->size(); ++i) {
////		pcl::PointXYZRGB& point = cloud_plane->points[i];
////		pcl::Normal& normal = cloud_normals->points[inliers->at(i)];
////		
////		// Assuming the floor is mostly vertical, you can check the normal's Z component
////		bool isFloor = normal.normal_z > 0.9;
////		
////		if (isFloor) {
////			// Set color for floor (e.g., green)
////			point.r = 0;
////			point.g = 255;
////			point.b = 0;
////		} else {
////			// Set color for walls (e.g., blue)
////			point.r = 0;
////			point.g = 0;
////			point.b = 255;
////		}
////	}
//
//
//	
//	// Save PointCloud to PCD file
//	pcl::io::savePCDFileASCII(std::string{MODELS_PATH} + "output_cloud.pcd", *cloud_plane);
//	
//	return 0;
//}
