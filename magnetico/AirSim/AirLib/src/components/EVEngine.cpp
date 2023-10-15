#include "components/EVEngine.hpp"
#include "components/Magnos.hpp"
#include "components/Battery.hpp"

namespace {
std::shared_ptr<Magnos> createGimbal(int id, std::shared_ptr<Node> parent, msr::airlib::Vector3r position){
	auto gimbal = std::make_shared<Magnos>();
	gimbal->loadData(id);
	gimbal->init();
	gimbal->attachPinball();
	gimbal->setPosition3D(position);
	parent->addChild(gimbal);
	return gimbal;
}



float voltsToCurrent(float voltage, float resistance) {
	if (resistance == 0.0f) {
		// Avoid division by zero
		return 0.0f;
	}
	
	return voltage / resistance;
}
}


EVEngine::EVEngine(){
	accelerating_ = false;
}

void EVEngine::init(){
	
	for(int i = 0; i<Settings::number_of_gimbals; ++i){
		gimbals_.push_back(createGimbal(i + 1, shared_from_this(), msr::airlib::Vector3r(0, 0, 0)));
	}
	
//	gimbals_.push_back(createGimbal(1, shared_from_this(), msr::airlib::Vector3r(-0.25, 0, 0)));
//
//	if(Settings::number_of_gimbals > 1){
//		gimbals_.push_back(createGimbal(2, shared_from_this(), msr::airlib::Vector3r(0.25, 0, 0)));
//	}
//
//	if(Settings::number_of_gimbals > 2){
//		gimbals_.push_back(createGimbal(3, shared_from_this(), msr::airlib::Vector3r(0, 0, -0.25)));
//	}
//
//	if(Settings::number_of_gimbals > 3){
//		gimbals_.push_back(createGimbal(4, shared_from_this(), msr::airlib::Vector3r(0, 0, 0.25)));
//	}
//
//	if(Settings::number_of_gimbals > 4){
//		gimbals_.push_back(createGimbal(5, shared_from_this(), msr::airlib::Vector3r(0, 0.25f, 0)));
//	}
//
//	if(Settings::number_of_gimbals > 5){
//		gimbals_.push_back(createGimbal(6, shared_from_this(), msr::airlib::Vector3r(0, -0.25f, 0)));
//	}
//
	battery_ = std::make_shared<RechargeableBattery>(Settings::battery_voltage, 
													 Settings::battery_voltage + 1, 
													 Settings::battery_voltage + 1, 
													 0.9f, 
													 0.95f);
	
	for(auto magnos : gimbals_){
		magnos->getCoilSystem().setDesignedEMFPerSecond(Settings::engine_voltage / Settings::number_of_gimbals);
	}

	
	std::function<void(int, float)> onVoltagePeak = [this](int index, float charge){	battery_->charge(voltsToCurrent(charge, 6), Settings::global_delta / 1000.0f);
		auto magnos = gimbals_[index];
		
		magnos->getCoilSystem().accumulator.discharge(charge);
	};
	
	for(auto magnos : gimbals_){
		magnos->getCoilSystem().setOnVoltagePeakCallback(onVoltagePeak);
	}
}

void EVEngine::update(float){
	
	Node::update(Settings::fixed_delta);
	
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
		
		float powerDraw = (totalCurrent / (float)gimbals_.size()) * totalDelta;
		
		for(auto magnos : gimbals_){
			auto _ = magnos->getCoilSystem().withdrawPower(powerDraw);
		}
	}
	float deltaTime = totalDelta;
	static float guiCounter = 0;
	static float guiEMF = 0;
	static float peakEMF = 0;
	
	guiCounter += deltaTime;
	
	float baseAccumulatedEMF = 0;
	float accumulatedEMF = 0;
	
	bool any_calibration = false;
	bool any_collection = false;
	
	for(auto magnos : gimbals_){
		
		//float inducedEMF = abs(magnos->getAlternatorSystem().emf);
		
		if(!magnos->getCoilSystem().calibrating()){
			baseAccumulatedEMF += magnos->getCoilSystem().lastBaseAccumulatedEMF;
			accumulatedEMF += magnos->getCoilSystem().lastAccumulatedEMF;
			//		recycledEMF = magnos->getCoilSystem().lastRecycledEMF;
		}
		
		if(magnos->getCoilSystem().calibrating()){
			any_calibration = true;
		}
		
		if(magnos->getCoilSystem().collecting()){
			any_collection = true;
		}
	}
	
	
	guiEMF = accumulatedEMF;
	
	if (guiEMF > peakEMF){
		if((!any_calibration) && !any_collection){
			peakEMF = guiEMF;
		} else {
			guiEMF = 0;
			peakEMF = 0;
		}
	}
	
	if(any_calibration || any_collection){
		guiCounter = 0;
		guiEMF = 0;
		peakEMF = 0;
		accumulatedEMF = 0;
		baseAccumulatedEMF = 0;
	}
	
	std::string status = "Running";
	if(any_calibration){
		status = "Calibrating";
	} else {
		if(any_collection){
			status = "Collecting Data";
		} else {
			status = "Running";
		}
	}
	// ImGui::Text("Input Voltage=%.4f", 1.5f);

	// feedback.input = peakEMF;
	feedback_.status = status;
	feedback_.peakEMF = peakEMF;

	feedback_.baseEMF = baseAccumulatedEMF;
	feedback_.EMF = accumulatedEMF;

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

float EVEngine::getBatteryVoltage() const {
	return battery_->getVoltage();
}

float EVEngine::accelerate(float){
	accelerating_ = true;
	
	float powerDraw = (2.5f / (float)gimbals_.size()) * Settings::fixed_delta;
	
	float totalPowerDrawn = 0;
	for(auto magnos : gimbals_){
		
		totalPowerDrawn += magnos->getCoilSystem().testWithdrawPower(powerDraw);
	}
	
	return totalPowerDrawn;

}

void EVEngine::decelerate(){
	accelerating_ = false;
}

const EVFeedback& EVEngine::getMagnosFeedback() const {
	return feedback_;
}
