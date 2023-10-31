#include "StbUtils.h"
#include "TusimpleEngine.h"

#include <string>
#include <iostream>

int main() {
	std::string workPath = MODELS_PATH; // You need to define MODELS_PATH
	
	std::string model_file = workPath + "fast_roadseg-tusimple.onnx";
	
	TusimpleEngine engine(model_file);
	
	// Load an input image
	std::string image_path = workPath + "input.jpg";
	
	auto input_data = load_image_data(image_path);
	
	try {
		auto _ = engine.detectLanes(input_data);
		
		std::cout << "Done!" << std::endl;
		
	} catch (const Ort::Exception& exception) {
		std::cout << "ERROR running model inference: " << exception.what() << std::endl;
		exit(-1);
	}
	
	return 0;
}
