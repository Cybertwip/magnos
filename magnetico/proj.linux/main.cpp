#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <filesystem>
#include "components/Magnos.hpp"
#include "components/EVEngine.hpp"
#include "components/Laser.hpp"

int main(int argc, char** argv)
{
    std::shared_ptr<EVEngine> engine = std::make_shared<EVEngine>();
    engine->setPosition3D(msr::airlib::Vector3r(0, 0, 0));
    engine->init("./");

    while (true)
    {
        engine->update(Settings::fixed_delta);

        // Wait for 1.0 / Settings::fixed_delta seconds
        std::this_thread::sleep_for(std::chrono::duration<float>(1.0 / Settings::fixed_delta));
    }

    return 0;
}
