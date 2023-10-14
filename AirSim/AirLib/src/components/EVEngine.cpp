#include "components/EVEngine.hpp"
#include "components/Magnos.hpp"
#include "components/Battery.hpp"

namespace {
std::shared_ptr<Magnos> createGimbal(int id, std::shared_ptr<Node> parent, msr::airlib::Vector3r position){
	auto gimbal = std::make_shared<Magnos>();
	gimbal->loadData(id);
	gimbal->init();
	gimbal->setPosition3D(position);
	parent->addChild(gimbal);
	gimbal->attachPinball();
	return gimbal;
}
}


EVEngine::EVEngine(){
	accelerating_ = false;
}

void EVEngine::init(){
	gimbals_.push_back(createGimbal(1, shared_from_this(), msr::airlib::Vector3r(-0.25, 0, 0)));
	
	if(Settings::number_of_gimbals > 1){
		gimbals_.push_back(createGimbal(2, shared_from_this(), msr::airlib::Vector3r(0.25, 0, 0)));
	}
	
	if(Settings::number_of_gimbals > 2){
		gimbals_.push_back(createGimbal(3, shared_from_this(), msr::airlib::Vector3r(0, 0, -0.25)));
	}
	
	if(Settings::number_of_gimbals > 3){
		gimbals_.push_back(createGimbal(4, shared_from_this(), msr::airlib::Vector3r(0, 0, 0.25)));
	}
	
	if(Settings::number_of_gimbals > 4){
		gimbals_.push_back(createGimbal(5, shared_from_this(), msr::airlib::Vector3r(0, 0.25f, 0)));
	}
	
	if(Settings::number_of_gimbals > 5){
		gimbals_.push_back(createGimbal(6, shared_from_this(), msr::airlib::Vector3r(0, -0.25f, 0)));
	}

	battery_ = std::make_shared<RechargeableBattery>(Settings::battery_voltage, 
													 Settings::battery_voltage + 1, 
													 Settings::battery_voltage + 1, 
													 0.9f, 
													 0.95f);
}

void EVEngine::update(){
	float totalDelta = Settings::global_delta / 1000.0f;
	
	float totalCurrent = 0.0f;
	float totalPower = 0.0f;
	float totalResistance = 0.0f;
	
	bool anyDataCollectionMode = false;
	for(auto magnos : gimbals_){
		
		magnos->update(totalDelta);
		
		totalCurrent += magnos->getCoilSystem().current;
		
		totalResistance += 1 * 6;
		
		if(!anyDataCollectionMode){
			anyDataCollectionMode = magnos->getCoilSystem().collecting();
		}
		
	}
	
	
	battery_->discharge(totalCurrent, totalDelta);


	if(accelerating_ || anyDataCollectionMode){
		
		float powerDraw = (2.5f / (float)gimbals_.size()) * totalDelta;
		
		float totalPowerDrawn = 0;
		for(auto magnos : gimbals_){
			
			totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
		}
	} 
	
	// if(car->anyLaserStatusOn() && enable_lasers){
		
	// 	float laserVoltage = 0;
		
	// 	for(auto laser : car->getLasers()){
	// 		laserVoltage += laser->getVoltageInput();
	// 	}
		
	// 	float powerDraw = (laserVoltage / (float)gimbals.size()) * totalDelta;
		
	// 	float totalPowerDrawn = 0;
	// 	for(auto gimbal : gimbals){
	// 		auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
			
	// 		totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
	// 	}
	// }
	
	
	// if(enable_lasers){
	// 	for(auto laser : car->getLasers()){
	// 		laser->update(totalDelta);
	// 	}
	// }
	
	// if(enable_lasers){
	// 	for(auto laser : car->getLasers()){
	// 		float storedPower = 0;
	// 		float powerToStore = (laser->getAccumulatedVoltage() / (float)gimbals.size()) * totalDelta;
	// 		for(auto gimbal : gimbals){
	// 			auto magnos = dynamic_cast<MaritimeGimbal3D*>(gimbal);
	// 			storedPower += magnos->getCoilSystem().storePower(powerToStore);
	// 		}
			
	// 		if(storedPower != 0){
	// 			laser->dischargeAccumulatedVoltage(storedPower);
	// 		}
			
	// 	}
		
	// }
}

float EVEngine::getVoltage() const {
	return battery_->getVoltage();
}

void EVEngine::accelerate(float){
	accelerating_ = true;
}

void EVEngine::decelerate(){
	accelerating_ = false;
}