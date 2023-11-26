#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"

int main(int argc, char** argv)
{
	
	EVEngine engine;
	
	engine.etPosition3D(msr::airlib::Vector3r(0, 0, 0));
	engine.init(std::filesystem::current_path().string());

    
    return 0;
}
