# include "onnx_to_mlpack.hpp"

int main()
{
	std::string workPath = MODELS_PATH;
	
	convertModel(workPath + "roadseg.onnx", workPath + "roadseg.xml", 384, 1248);
	
	return 0;
}

