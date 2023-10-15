#include "components/systems/CoilSystem.hpp"

#include <mlpack.hpp>

static long long timePrev = 0;
static long long timeNow = 0;

CoilSystem::CoilSystem(int id, float voltage, float resistance, float current, float coilTurns)
: coilResistance(resistance), maxCurrent(current), turns(coilTurns), coil_id(id),
filterBase(VoltageController(32, Settings::desired_base_voltage, Settings::desired_base_voltage, false)),
filterIncrease(VoltageController(32, Settings::desired_capacitor_voltage, Settings::desired_capacitor_voltage, true)){
		
    setCurrentFromVoltage(voltage);
	std::string home = getenv("HOME");

    hasML = loadDataAndTrainModel(home + "/calibration" + "_" + std::to_string(coil_id) + ".bin");
    
    if(hasML){
		data_collection_mode = false;
    }

    this->recalibrate();
    
    filterIncrease.setOnCapacitorCharged([this](float charge){
        if(!calibrating()){
            this->recycledEMF += charge;
        }
    });
}

CoilSystem::~CoilSystem(){
}

float CoilSystem::withdrawPower(float power) {
	if (power <= 0) {
		// No valid power to withdraw, return 0
		return 0.0f;
	}
	
	// Calculate the amount that can be withdrawn, ensuring it doesn't go below zero
	float withdrawn = std::min(power, accumulator.getVoltage());
	
	// Subtract the withdrawn power from accumulatedEMF
	accumulator.discharge(withdrawn);
	
	return withdrawn;
}

float CoilSystem::storePower(float power) {
	if (power <= 0) {
		// No valid power to store, return 0
		return 0.0f;
	}
	
	// Calculate the amount that can be stored without exceeding desiredEMFPerSecond
	float availableCapacity = accumulator.getCapacity() - accumulator.getVoltage();
	float stored = std::min(power, availableCapacity);
	
	// Add the stored power to accumulatedEMF
	accumulator.charge(stored);
	
	return stored;
}

void CoilSystem::recalibrate(){
	pidCurrent = PIDController(1.0f, 0, 0);

	pidCurrent.startAutoTuning(maxCurrent, 0);
	calibration = true;
	
	resetAccumulators();
}

void CoilSystem::setDesignedEMFPerSecond(float desiredEMF){
	designedEMFPerSecond = desiredEMF;
	resetAccumulators();
}

void CoilSystem::setOnVoltagePeakCallback(std::function<void(int, float)> onVoltagePeak){
	this->onVoltagePeak = onVoltagePeak;
}

void CoilSystem::resetAccumulators(){
	accumulationTime = 0.0f;
	accumulator = Capacitor(designedEMFPerSecond); // @TODO make it fixed
	baseAccumulatedEMF = 0;
	lastBaseAccumulatedEMF = 0;
	lastAccumulatedEMF = 0;
	
	timePrev = 0;
	timeNow = 0;
	
	filterBase = VoltageController(32, Settings::desired_base_voltage, Settings::desired_base_voltage, false);
	
	filterIncrease = VoltageController(32, designedEMFPerSecond, Settings::desired_capacitor_voltage, true);
	
	this->current = 0;

}

void CoilSystem::saveDataToBinary(const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    
    for (const DataPoint& data : dataCollection) {
        outFile.write(reinterpret_cast<const char*>(&data), sizeof(DataPoint));
    }
    
    outFile.close();
}

bool CoilSystem::loadDataAndTrainModel(const std::string& filename) {
    // Check if the file exists
    std::ifstream testFile(filename);
    if (!testFile) {
        std::cerr << "File does not exist: " << filename << std::endl;
        return false;  // return unsuccessful load and training
    }
    testFile.close();
    
    std::ifstream inFile(filename, std::ios::binary);
    
    std::vector<DataPoint> loadedData;
    
    while (inFile) {
        DataPoint data;
        inFile.read(reinterpret_cast<char*>(&data), sizeof(DataPoint));
        if(inFile) {
            loadedData.push_back(data);
        }
    }
    
    inFile.close();
    
    dataCollection = loadedData;
    
    // Convert loadedData to arma::mat format for mlpack
    arma::mat dataset(4, loadedData.size());
    
    for (size_t i = 0; i < loadedData.size(); i++) {
        dataset(0, i) = loadedData[i].emfError;
        dataset(1, i) = loadedData[i].desiredCurrent;
        dataset(2, i) = loadedData[i].currentAdjustment;
		dataset(3, i) = loadedData[i].finalCurrent;
    }
    
    // Assuming last row (currentAdjustment) as labels and others as features
    arma::mat labels = dataset.row(3);
    dataset.shed_row(3);
    
    // Train a linear regression model
    mlModel = std::make_unique<mlpack::LinearRegression>(dataset, labels);
    
    return true;  // return successful load and training
}


void CoilSystem::setCurrentFromVoltage(float voltage) {
    this->current = voltage / (coilResistance * numberOfCoils);
    this->current = std::clamp(this->current, 0.0f, maxCurrent);  // Ensure it doesn't exceed maxCurrent
}

msr::airlib::Vector3r CoilSystem::computeMagneticField(CoilEntity::AttachedEntity& coil, const msr::airlib::Vector3r& origin, const msr::airlib::Vector3r& point, MagnetPolarity polarity) const
{
	msr::airlib::Vector3r direction = (point - origin).normalized();
	float distance = sqrt(pow(origin.x() - point.x(), 2) + pow(origin.y() - point.y(), 2) + pow(origin.z() - point.z(), 2));
    float magnitude = (this->current * coil.turns) / (2 * M_PI * distance);
    
    // Reverse the direction if the polarity is SOUTH
    if(polarity == MagnetPolarity::SOUTH) {
        magnitude = -magnitude;
    }
    
    return direction * magnitude;
}


msr::airlib::Vector3r CoilSystem::combineFieldsOrForces(const msr::airlib::Vector3r& origin) {
	msr::airlib::Vector3r totalField(0, 0, 0);
    
    for (auto& coil : _attachedEntities) {
		msr::airlib::Vector3r field = computeMagneticField(coil, origin, coil.position, coil.polarity);
        totalField += field;
    }
    
    return totalField;
}

std::shared_ptr<Node> CoilSystem::attach(float radius, MagnetDirection direction, MagnetPolarity polarity) {
    
	msr::airlib::Vector3r position(0, 0, 0);
    
    switch (direction) {
        case MagnetDirection::NORTH:
            position = msr::airlib::Vector3r(0, radius, 0);
            break;
        case MagnetDirection::NORTHEAST:
            position = msr::airlib::Vector3r(radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::EAST:
            position = msr::airlib::Vector3r(radius, 0, 0);
            break;
        case MagnetDirection::SOUTHEAST:
            position = msr::airlib::Vector3r(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::SOUTH:
            position = msr::airlib::Vector3r(0, -radius, 0);
            break;
        case MagnetDirection::SOUTHWEST:
            position = msr::airlib::Vector3r(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::WEST:
            position = msr::airlib::Vector3r(-radius, 0, 0);
            break;
        case MagnetDirection::NORTHWEST:
            position = msr::airlib::Vector3r(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::FRONT:
            position = msr::airlib::Vector3r(0, 0, -radius);
            break;
            
        case MagnetDirection::BACK:
            position = msr::airlib::Vector3r(0, 0, radius);
            break;

    }
    
    AttachedEntity coilEntity = {position, polarity, 0, turns};
	
	_attachedEntities.push_back(coilEntity);
	
	auto coil = std::make_shared<Coil>(_attachedEntities, _attachedEntities.size());
	
	coil->setPosition3D(position);
	
	coils.push_back(coil);
	
	return coil;
}

bool CoilSystem::calibrating() {
	return calibration;
}

bool CoilSystem::collecting() {
	return data_collection_mode || schedule_recalibration_for_collection;
}

void CoilSystem::scheduleCollection(){
	schedule_recalibration_for_collection = true;
}

void CoilSystem::adjustCurrentBasedOn(float) {
    // Calculate desired current based on EMF error and resistance
    float emfError = accumulator.getCapacity() - accumulator.getVoltage();
    
    // Compute error for the PID
    float desiredCurrent = emfError / totalResistance;
	
    float currentAdjustment = 0;

	float fixedDelta = (timeNow - timePrev) / 1000.0f;
	
    if(pidCurrent.calibrate(emfError / accumulator.getCapacity(), desiredCurrent, fixedDelta, timeNow)){
                                
        currentAdjustment = pidCurrent.getRelayState() ? 0 : maxCurrent;
        
    } else {
        
        currentAdjustment = pidCurrent.compute(desiredCurrent, fixedDelta);

        currentAdjustment = std::clamp(this->current + currentAdjustment, 0.0f, maxCurrent);

        if(calibration){
            calibration = false;
			          
			accumulator = Capacitor(designedEMFPerSecond); // @TODO go to max voltage

			data_collection_mode = false;
			
			if(!hasML || schedule_data_collection_mode){
				schedule_data_collection_mode = false;
				data_collection_mode = true;
			
				this->resetAccumulators();

			}
			
			
			this->current = 0;

			return;

        }
		
        
        if(hasML && !data_collection_mode){
            // Check if conditions are met to retrain
            if(fabs(emfError) > Settings::error_trial && !schedule_data_collection_mode) {
                
				//Settings::schedule_recalibration_for_collection = true;
            }
			
			desiredCurrent = std::clamp(desiredCurrent, 0.0f, maxCurrent);

            // Create a column vector for input features
            arma::mat input(3, 1);
            input(0, 0) = emfError;            // This should be your new emfError value
            input(1, 0) = desiredCurrent;         // Replace with your new desiredCurrent value
			input(2, 0) = currentAdjustment;      // Replace with your new currentAdjustment value

            arma::rowvec output;  // Use rowvec instead of mat
            mlModel->Predict(input, output);
            
            // Now `output` contains the predicted finalCurrent value
            float predictedFinalCurrent = output(0, 0);
            
			predictedFinalCurrent = std::clamp(predictedFinalCurrent, 0.0f, maxCurrent);

            this->current = predictedFinalCurrent;
			
			currentAdjustment = this->current;
        }
		
    }

    this->current = currentAdjustment;
}

void CoilSystem::update(){
    for(auto coil : coils){
        coil->updatePositions();
    }
}

void CoilSystem::update(float measuredEMF, float delta) {
	accumulator.charge(fabs(measuredEMF));
	
	accumulationTime += delta;
	
	if(!calibrating()){
		baseAccumulatedEMF = filterBase.controlVoltage(accumulator.getVoltage());
	}
	
	timePrev = timeNow;
	timeNow += delta;

	if((accumulator.getVoltage() != accumulator.getCapacity() && !calibrating()) || (calibrating() && accumulationTime >= Settings::global_delta * 60)){
		if(!data_collection_mode){
			adjustCurrentBasedOn(accumulationTime);
		}
    }
	
	if(data_collection_mode && !calibrating()){
		
		// Calculate desired current based on EMF error and resistance
		float emfError = accumulator.getCapacity() - accumulator.getVoltage();
		
		if(emfError != 0.0f){
			
			// Compute error for the PID
			float desiredCurrent = emfError / totalResistance;
			
			float currentAdjustment = 0;
			
			float fixedDelta = (timeNow - timePrev) / 1000.0f;
			
			currentAdjustment = pidCurrent.compute(desiredCurrent, fixedDelta);
			
			currentAdjustment = std::clamp(this->current + currentAdjustment, 0.0f, maxCurrent);
			
			DataPoint point;
			point.emfError = emfError;
			point.desiredCurrent = desiredCurrent;
			point.currentAdjustment = currentAdjustment;
			point.finalCurrent = this->current;
			
			dataCollection.push_back(point);
			
			if((dataCollection.size() * sizeof(DataPoint)) % Settings::data_collection_bin_size == 0){
				std::string home = getenv("HOME");
				
				saveDataToBinary(home + "/calibration" + "_" + std::to_string(coil_id) + ".bin");
				
				data_collection_mode = false;
				
				hasML = loadDataAndTrainModel(home + "/calibration" + "_" + std::to_string(coil_id) + ".bin");
				
				this->recalibrate();
				
				return;
				
			}
			
			this->current = currentAdjustment;
		} else {
			if(accumulator.getVoltage() >= accumulator.getCapacity()){
				
				if(onVoltagePeak){
					onVoltagePeak(coil_id - 1, accumulator.getVoltage());
				}

			}


		}
		
	}
	
	if(accumulationTime == Settings::global_delta * 60){
		accumulationTime = 0.0f;

		if(accumulator.getVoltage() > accumulator.getCapacity()){
			
//			if(!calibrating()){
//				accumulatedEMF = filterIncrease.controlVoltage(accumulatedEMF);
//			}
//
		} else if (accumulator.getVoltage() < accumulator.getCapacity() && !calibrating()){
			float storedVoltage = filterIncrease.getCapacitorVoltage();
			
//			 try to keep up
			float recycledConsumption = filterIncrease.consumeFromCapacitor(std::min(storedVoltage, accumulator.getCapacity() - accumulator.getVoltage()));
			
			accumulator.charge(recycledConsumption);
		}

		lastBaseAccumulatedEMF = baseAccumulatedEMF;
		lastAccumulatedEMF = accumulator.getVoltage();
		lastRecycledEMF = recycledEMF;
		
		if(accumulator.getVoltage() >= accumulator.getCapacity() || (accumulator.getVoltage() >= accumulator.getCapacity() && calibrating())){

			if(onVoltagePeak){
				onVoltagePeak(coil_id - 1, accumulator.getVoltage());
			}
			
			//accumulator.discharge(accumulator.getCapacity());
		}
	}

	
	if(schedule_recalibration_for_collection){
		schedule_recalibration_for_collection = false;
		schedule_data_collection_mode = true;
		data_collection_mode = false;
		
		this->recalibrate();
	}

}
