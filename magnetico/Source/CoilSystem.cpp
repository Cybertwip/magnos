#include "CoilSystem.h"

CoilSystem::CoilSystem(float voltage, float resistance, float current, float coilTurns)
	: coilResistance(resistance), maxCurrent(current), turns(coilTurns){
    //setCurrentFromVoltage(voltage);
            
    // Get current time in seconds since epoch
    auto now = std::chrono::system_clock::now();
    auto epochTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    
    previousTime = epochTime;
    
    nowTime = epochTime;

    std::string home = getenv("HOME");

    hasML = loadDataAndTrainModel(home + "/calibration.bin");
    
    if(!hasML){
		Settings::data_collection_mode = true;
		Settings::cycles_per_collection = data_collection_mode_cycles;
    } else {
		Settings::data_collection_mode = false;
		Settings::cycles_per_collection = 1;
	}

    loadPID(home + "/pid.bin");

    pidCurrent.startAutoTuning(maxCurrent, 0);
    
    this->current = 0;

    filterIncrease.setOnCapacitorCharged([this](float charge){
        if(!calibrating() && !adapting()){
            this->recycledEMF += charge;
        }
    });

}

CoilSystem::~CoilSystem(){
    std::string home = getenv("HOME");
    savePIDToBinary(home + "/pid.bin");
}

void CoilSystem::saveDataToBinary(const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    
    for (const DataPoint& data : dataCollection) {
        outFile.write(reinterpret_cast<const char*>(&data), sizeof(DataPoint));
    }
    
    outFile.close();
}

void CoilSystem::savePIDToBinary(const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    
    outFile.write(reinterpret_cast<const char*>(&trainedPID), sizeof(trainedPID));
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
    mlModel = std::make_unique<mlpack::regression::LinearRegression>(dataset, labels);
    
    return true;  // return successful load and training
}


bool CoilSystem::loadPID(const std::string& filename) {
    // Check if the file exists
    std::ifstream testFile(filename);
    if (!testFile) {
        std::cerr << "File does not exist: " << filename << std::endl;
        return false;  // return unsuccessful load and training
    }
    testFile.close();
    
    std::ifstream inFile(filename, std::ios::binary);
            
    PID data;
    inFile.read(reinterpret_cast<char*>(&data), sizeof(PID));
    inFile.close();
    
    pidCurrent = PIDController(data.kp, data.ki, data.kd);
    
    return true;  // return successful load and training
}


void CoilSystem::setCurrentFromVoltage(float voltage) {
    this->current = voltage / (coilResistance * numberOfCoils);
    this->current = std::clamp(this->current, 0.0f, maxCurrent);  // Ensure it doesn't exceed maxCurrent
}

ax::Vec3 CoilSystem::computeMagneticField(CoilEntity::AttachedEntity& coil, const ax::Vec3& point, MagnetPolarity polarity) const {
    ax::Vec3 direction = (point).getNormalized();
    float distance = ax::Vec3(0, 0, 0).distance(point);
    float magnitude = (this->current * coil.turns) / (2 * M_PI * distance);
    
    // Reverse the direction if the polarity is SOUTH
    if(polarity == MagnetPolarity::SOUTH) {
        magnitude = -magnitude;
    }
    
    return direction * magnitude;
}


ax::Vec3 CoilSystem::combineFieldsOrForces() {
    ax::Vec3 totalField(0, 0, 0);
    
    for (auto& coil : _attachedEntities) {
        ax::Vec3 field = computeMagneticField(coil, coil.position, coil.polarity);
        totalField += field;
    }
    
    return totalField;
}

void CoilSystem::attachToDisk(ax::Node* node, float radius, MagnetDirection direction, MagnetPolarity polarity) {
    
    ax::Vec3 position(0, 0, 0);
    
    switch (direction) {
        case MagnetDirection::NORTH:
            position = ax::Vec3(0, radius, 0);
            break;
        case MagnetDirection::NORTHEAST:
            position = ax::Vec3(radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::EAST:
            position = ax::Vec3(radius, 0, 0);
            break;
        case MagnetDirection::SOUTHEAST:
            position = ax::Vec3(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::SOUTH:
            position = ax::Vec3(0, -radius, 0);
            break;
        case MagnetDirection::SOUTHWEST:
            position = ax::Vec3(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::WEST:
            position = ax::Vec3(-radius, 0, 0);
            break;
        case MagnetDirection::NORTHWEST:
            position = ax::Vec3(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::FRONT:
            position = ax::Vec3(0, 0, -radius);
            break;
            
        case MagnetDirection::BACK:
            position = ax::Vec3(0, 0, radius);
            break;

    }
    
    AttachedEntity coilEntity = {position, polarity, 0, turns};
    _attachedEntities.emplace_back(coilEntity);
    
    ax::Mesh* magnetMesh = createCube(radius / 16);
    auto magnetRenderer = ax::MeshRenderer::create();
    magnetRenderer->addMesh(magnetMesh);
    magnetRenderer->setPosition3D(ax::Vec3(0, 0, 0));
    magnetRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
    magnetRenderer->setTexture("black.jpg");
    
    auto coil = Coil::create(_attachedEntities, _attachedEntities.size());
    
    coil->setPosition3D(position);
    coil->addChild(magnetRenderer);
    
    node->addChild(coil);

    coils.push_back(coil);
}

bool CoilSystem::calibrating() {
    return calibration;
}

bool CoilSystem::adapting() {
    return adaptive;
}

void CoilSystem::adjustCurrentBasedOn(float dt) {
    desiredEMFPerSecond = Settings::desired_voltage_increase_per_second;
    // Calculate desired current based on EMF error and resistance
    float emfError = Settings::desired_voltage_increase_per_second - accumulatedEMF;
    
    // Get current time in seconds since epoch
    auto now = std::chrono::system_clock::now();
    auto epochTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    
    previousTime = nowTime;
    
    nowTime = epochTime;

    // Compute error for the PID
    float desiredCurrent = emfError / totalResistance;

    float currentAdjustment = 0;
    
    static long long simulatedEpoch = 0;
    
    simulatedEpoch += 1000;
    
    if(pidCurrent.calibrate(desiredCurrent, dt, simulatedEpoch)){
                                
        currentAdjustment = pidCurrent.getRelayState() ? maxCurrent : 0;
        
    } else {
        
        currentAdjustment = pidCurrent.compute(desiredCurrent, dt);

        currentAdjustment = std::clamp(this->current + currentAdjustment, 0.0f, maxCurrent);

        if(calibration){
            calibration = false;
            
            std::string home = getenv("HOME");
            savePIDToBinary(home + "/pid.bin");

            if(adaptive_calibration){
                desiredEMFPerSecond = adaptive_calibration_voltage;
            } else {
                desiredEMFPerSecond = Settings::desired_voltage_increase_per_second;
            }
        }
        
        
        
        if(Settings::data_collection_mode && !calibrating() && !adapting()){
			DataPoint point;
			point.emfError = emfError;
			point.desiredCurrent = desiredCurrent;
			point.currentAdjustment = currentAdjustment;
			point.finalCurrent = this->current;
			
			dataCollection.push_back(point);

			if(dataCollection.size() * sizeof(DataPoint) >= data_collection_bin_size){
				std::string home = getenv("HOME");
				
				saveDataToBinary(home + "/calibration.bin");
				
				exit(0);
			}
        }
        
        if(hasML && !Settings::data_collection_mode){
            // Check if conditions are met to retrain
            if(fabs(emfError) > error_trial) {
                //retrainModel();
                
                if(adaptive_calibration){
                    // Calibration upwards
                    if(isCalibratingUpwards && accumulatedEMF + emfError >= adaptive_calibration_voltage){
                        desiredEMFPerSecond = Settings::desired_voltage_increase_per_second;
                        isCalibratingUpwards = false;  // Switch calibration direction
                        adaptive = true;
                    }
                    // Calibration downwards
                    else if(!isCalibratingUpwards && accumulatedEMF + emfError <= Settings::desired_voltage_increase_per_second){
                        desiredEMFPerSecond = adaptive_calibration_voltage;
                        isCalibratingUpwards = true;   // Switch calibration direction
                        adaptive = true;
                    }
                }
            } else {
                adaptive = false;
            }
            // Create a column vector for input features
            arma::mat input(3, 1);
            input(0, 0) = emfError;            // This should be your new emfError value
            input(1, 0) = desiredCurrent;         // Replace with your new desiredCurrent value
            input(2, 0) = currentAdjustment;      // Replace with your new currentAdjustment value
            
            arma::rowvec output;  // Use rowvec instead of mat
            mlModel->Predict(input, output);
            
            // Now `output` contains the predicted finalCurrent value
            float predictedFinalCurrent = output(0, 0);
            
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
    accumulatedEMF += fabs(measuredEMF);
    if(!calibrating() && !adapting()){

        guiAccumulatedEMF += fabs(measuredEMF);
        
    }

    // Get current time in seconds since epoch
    auto now = std::chrono::system_clock::now();
    auto epochTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    
    
    if(accumulationTime == 0){
        previousTime = nowTime;
    }

    accumulationTime += delta;
    guiAccumulationTime += delta;

    baseAccumulatedEMF = filterBase.controlVoltage(accumulatedEMF);
	
    if(accumulatedEMF >= desiredEMFPerSecond || ((accumulationTime >= 1.0f) && calibrating())){
        
        nowTime = epochTime;

        adjustCurrentBasedOn(global_delta * calibration_time);
        
        accumulationTime = 0.0f;
                    
        if(!calibrating() && !adapting()){
            accumulatedEMF = filterIncrease.controlVoltage(accumulatedEMF);
        } else {
            accumulatedEMF = 0;
        }

    } else if (accumulatedEMF < desiredEMFPerSecond && accumulationTime >= 1.0f){
		if(!calibrating() && !adapting()){
			adjustCurrentBasedOn(global_delta * calibration_time);
			accumulationTime = 0.0f;
			accumulatedEMF = filterIncrease.controlVoltage(accumulatedEMF);
		}
	}
    
    if(!calibrating() && !adapting()){
        guiBaseAccumulatedEMF = baseAccumulatedEMF;
        guiAccumulatedEMF = accumulatedEMF;
        
        if(guiAccumulatedEMF >= desiredEMFPerSecond){
            
            guiAccumulationTime = 0;
            lastBaseAccumulatedEMF = guiBaseAccumulatedEMF;
            lastAccumulatedEMF = guiAccumulatedEMF;
            lastRecycledEMF = recycledEMF;
            
            guiAccumulatedEMF = 0;
			accumulatedEMF = 0;

            
            accumulatedEMF = 0;
        }

    }
}
