#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <thread>
#include "StateEstimator.h"
#include <QString>

// Define a struct to hold motor data
struct MotorData {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> torques;
    std::vector<double> imu; 
};

std::vector<MotorData> readMotorData(const std::string& file_path); 

int main() {
    double time = 0.0;
    int count = 0;
    // Create an instance of the StateEstimator
    Estimator estimator;
    
    // Initialize the estimator
    QString path = "../model.json"; 
    estimator.init(path, 0.0025);

    // Open the file containing motor data
    QString input_path = "../inputdata.txt";
    std::ifstream motorDataFile(input_path.toStdString());
    std::string line;
    if (!motorDataFile.is_open()) {
        std::cerr << "Failed to open MotorData.txt" << std::endl;
        return -1;
    }

    // Initialise vectors for 12 motors
    Eigen::VectorXd motorPositions = Eigen::VectorXd::Zero(12); 
    Eigen::VectorXd motorVelocities = Eigen::VectorXd::Zero(12); 
    Eigen::VectorXd motorAccelerations = Eigen::VectorXd::Zero(12); 
    Eigen::VectorXd motorTorques = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd imudata = Eigen::VectorXd::Zero(9);
    // Store the estimation results
    Eigen::MatrixXd estState = Eigen::MatrixXd::Zero(12, 6);
    std::vector<MotorData> motordata = readMotorData(input_path.toStdString()); 

    std::cout << "Starting state estimation test..." << std::endl;

    while (getline(motorDataFile, line)) {
        // Parse the line to fill Eigen vectors
        // parseMotorData(line, motorPositions, motorVelocities, motorTorques); 

        for (int i = 0; i < 12; ++i) {
            motorPositions(i) = motordata[count].positions[i];
            motorVelocities(i) = motordata[count].velocities[i];
            motorTorques(i) = motordata[count].torques[i];
        }
        for (int i = 0; i < 9; ++i) {
            imudata(i) = motordata[count].imu[i];
        }

        // Since the accelerations cannot be read from the motor, just set it to 0
        std::cout << "-------------------------------INPUT--------------------------------" << std::endl;
        std::cout << "Motor Positions: " << motorPositions.transpose() << std::endl;
        std::cout << "Motor Velocities: " << motorVelocities.transpose() << std::endl;
        std::cout << "Motor Torques: " << motorTorques.transpose() << std::endl;
        std::cout << "imu data: " << imudata.transpose() << std::endl; 

        // Run the estimator, store the results
        estimator.run(motorPositions, motorVelocities, motorAccelerations, motorTorques, imudata); 
        estState = estimator.get_state(); 
        // Print the results
        std::cout << "-------------------------------OUTPUT-------------------------------" << std::endl;
        // std::cout << "Estimated state: " << std::endl << estState << std::endl; 
        std::cout << "Time: " << time << " s" << std::endl;
        std::cout << "Body State: " << std::endl << estState.block(0, 0, 4, 6) << std::endl; 
        std::cout << "Left Foot State: " << std::endl << estState.block(4, 0, 4, 6) << std::endl;
        std::cout << "Right Foot State: " << std::endl << estState.block(8, 0, 4, 6) << std::endl;
        std::cout << std::endl;

        // Sleeping for 2.5ms for a 400 Hz update rate as given in the problem statement
        time += 0.0025; 
        count++; 
        std::this_thread::sleep_for(std::chrono::microseconds(2500));
    }

    motorDataFile.close();
    
    std::cout << "State estimator test completed." << std::endl;
    
    return 0;
}


// Function to read and parse the MotorData.txt file
std::vector<MotorData> readMotorData(const std::string& file_path) {
    std::ifstream file(file_path);
    std::vector<MotorData> motor_data;

    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            MotorData data;
            double value;

            // Skip the first element (time)
            iss >> value;
            // Read motor positions
            for (int i = 0; i < 12; ++i) {
                iss >> value;
                data.positions.push_back(value);
            }
            // Read motor velocities
            for (int i = 0; i < 12; ++i) {
                iss >> value;
                data.velocities.push_back(value);
            }
            // Read motor torques
            for (int i = 0; i < 12; ++i) {
                iss >> value;
                data.torques.push_back(value);
            }
            // Read imu data
            for (int i = 0; i < 9; ++i) {
                iss >> value;
                data.imu.push_back(value);
            }
            motor_data.push_back(data);
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << file_path << std::endl;
    }

    return motor_data;
}