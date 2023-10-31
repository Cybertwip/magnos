# include "onnx_to_mlpack.hpp"

int main()
{
	std::string workPath = MODELS_PATH;
	
	convertModel(workPath + "roadseg.onnx", workPath + "roadseg.bin", 640, 360);
	
	return 0;
}

