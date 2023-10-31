#include "StbUtils.h"
#include "TusimpleEngine.h"

#include <string>
#include <iostream>

int main() {
	TusimpleEngine engine;
	
	 std::string image_path = std::string{MODELS_PATH} + "input.jpg";
	
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
