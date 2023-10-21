#include "components/EVEngine.hpp"
#include "components/Magnos.hpp"
#include "components/Battery.hpp"
#include "components/Laser.hpp"

float EVEngine::max_voltage = 400;

namespace {
std::shared_ptr<Magnos> createGimbal(int id, std::shared_ptr<msr::airlib::Node> parent, msr::airlib::Vector3r position){
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

float currentToVolts(float current, float resistance) {
	if (resistance <= 0.0f) {
		std::cerr << "Error: Resistance must be greater than 0." << std::endl;
		return 0.0f; // You can choose how to handle the error.
	}
	
	return current * resistance;
}

}


EVEngine::EVEngine(float max_engine_voltage) : max_engine_voltage_(max_engine_voltage){
	
	EVEngine::max_voltage = max_engine_voltage_;
	
	accelerating_ = false;
	engine_consumption_ = 0.0f;
}

void EVEngine::init(){
	
	for(int i = 0; i<Settings::number_of_gimbals; ++i){
		gimbals_.push_back(createGimbal(i + 1, shared_from_this(), msr::airlib::Vector3r(0, 0, 0)));
	}
	battery_ = std::make_shared<RechargeableBattery>(Settings::battery_voltage, 
													 Settings::battery_voltage + 1, 
													 Settings::battery_voltage + 1, 
													 0.8f,
													 0.95f);
	
	for(auto magnos : gimbals_){
		magnos->getCoilSystem().setDesignedEMFPerSecond(max_engine_voltage_ / Settings::number_of_gimbals);
	}

	std::function<void(int, float)> onVoltagePeak = [this](int index, float charge){	battery_->charge(voltsToCurrent(charge, Settings::circuit_resistance) / Settings::fixed_delta, Settings::fixed_delta);
		auto magnos = gimbals_[index];
		
		magnos->getCoilSystem().accumulator.discharge(charge);
	};
	
	for(auto magnos : gimbals_){
		magnos->getCoilSystem().setOnVoltagePeakCallback(onVoltagePeak);
	}
	
	for(int i = 0; i<Settings::number_of_lasers; ++i){
		lasers_.push_back(std::make_shared<Laser>(0.02f, true, 0.1f, 0.0f, 5000));
	}
}

bool EVEngine::isCalibrating(){
	bool calibrating = false;
	
	for(auto magnos : gimbals_){
		
		calibrating = magnos->getCoilSystem().calibrating();
		
		if(calibrating){
			break;
		}
	}
	
	return calibrating;
}

void EVEngine::setEngineConsumption(float voltage){
	engine_consumption_ = voltage;
}

float EVEngine::getEngineConsumption() const {
	return engine_consumption_;
}

void EVEngine::update(float){
	
	msr::airlib::Node::update(Settings::fixed_delta);
	
	if(Settings::enable_lasers){
		for(auto laser : lasers_){
			laser->update(Settings::fixed_delta);
		}
	}
			
	if(Settings::enable_lasers){
		float laserVoltage = Settings::desired_laser_voltage;
		float powerDraw = (laserVoltage / Settings::number_of_gimbals) * Settings::fixed_delta;
		
		float totalPowerDrawn = 0;
		for(auto magnos : gimbals_){
			totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
		}
			
		float laserInput = totalPowerDrawn / Settings::fixed_delta;
		
		for(auto laser : lasers_){
			laser->setVoltageInput(laserInput);
		}
		
		battery_->discharge(voltsToCurrent(laserInput, 6), Settings::fixed_delta);
	}
	
	bool anyDataCollectionMode = false;
	for(auto magnos : gimbals_){
		
		magnos->update(Settings::fixed_delta);
		
		float totalPowerDrawn = 0.0f;
		
		totalPowerDrawn += currentToVolts(magnos->getCoilSystem().current, Settings::circuit_resistance) * Settings::fixed_delta;

		battery_->discharge(voltsToCurrent(totalPowerDrawn / Settings::fixed_delta, Settings::circuit_resistance), Settings::fixed_delta);
				
		if(!anyDataCollectionMode){
			anyDataCollectionMode = magnos->getCoilSystem().collecting();
		}
	}
	
	if(accelerating_ || anyDataCollectionMode){
		float powerDraw = 0;
		
		powerDraw += engine_consumption_;
		
		powerDraw /= Settings::number_of_gimbals;
		
		powerDraw *= Settings::fixed_delta;
		
		float totalPowerDrawn = 0.0f;
		for(auto magnos : gimbals_){
			totalPowerDrawn += magnos->getCoilSystem().withdrawPower(powerDraw);
		}
		
		battery_->discharge(voltsToCurrent(totalPowerDrawn / Settings::fixed_delta, Settings::circuit_resistance), Settings::fixed_delta);
	}
	
	static float guiEMF = 0;
	static float peakEMF = 0;
		
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
	
	if(Settings::enable_lasers){
		for(auto laser : lasers_){
			float storedPower = laser->getAccumulatedVoltage();
			
			if(storedPower != 0){
				laser->dischargeAccumulatedVoltage(storedPower);
			}
			
			battery_->charge(voltsToCurrent(storedPower, Settings::circuit_resistance) / Settings::fixed_delta, Settings::fixed_delta);
		}
		
	}
}

float EVEngine::getBatteryVoltage() const {
	return battery_->getVoltage();
}

float EVEngine::accelerate(float throttle){
	accelerating_ = true;
	
	float totalDelta = Settings::fixed_delta;

	float powerDraw = 0;
	
	powerDraw += engine_consumption_;
	
	powerDraw /= Settings::number_of_gimbals;
	
	powerDraw *= totalDelta;
	
	powerDraw *= throttle;
	
	float totalPowerDrawn = 0.0f;
	for(auto magnos : gimbals_){
		totalPowerDrawn += magnos->getCoilSystem().testWithdrawPower(powerDraw);
		totalPowerDrawn += currentToVolts(magnos->getCoilSystem().current, Settings::circuit_resistance);
	}
	
	return totalPowerDrawn;
}

void EVEngine::decelerate(){
	accelerating_ = false;
}

const EVFeedback& EVEngine::getMagnosFeedback() const {
	return feedback_;
}
