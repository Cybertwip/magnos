#include "StbUtils.h"
//#include "HybridnetsEngine.h"

#include "TusimpleEngine.h"

#include <string>
#include <iostream>

int main() {
	//HybridnetsEngine engine(3);
	TusimpleEngine engine;
		
	std::string input_path = std::string{MODELS_PATH} + "input.png";
	std::string output_path = std::string{MODELS_PATH} + "output.png";

	auto input_data = load_image_data(input_path);
	
	try {
		auto [processedImage, points, lanes] = engine.detectLanes(input_data);
		
		saveImage(processedImage, output_path);

		std::cout << "Done!" << std::endl;
		
	} catch (const Ort::Exception& exception) {
		std::cout << "ERROR running model inference: " << exception.what() << std::endl;
		exit(-1);
	}
	
	return 0;
}
