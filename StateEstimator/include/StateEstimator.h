#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include "StateData.h"
#include "Basics.h"
#include "VelKalmanFilter.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>

/**
 * @brief The Estimator class represents a state estimator for a robot.
 * 
 * This class is responsible for estimating the state of a robot based on the provided sensor data.
 * It uses the RBDL library for robot dynamics calculations and provides methods to initialize the estimator,
 * run the estimation process, and retrieve the estimated state.
 */
class Estimator
{
public: 
    Estimator();
    ~Estimator(); 

    /**
     * @brief Initializes the estimator with the robot model.
     * @param model_path The path to the robot model file.
     * @param dt The time step for the state estimation process.
     */
    void init(QString model_path, double dt); 

    /**
     * @brief Runs the state estimation process.
     * @param q_a The joint positions.
     * @param qd_a The joint velocities.
     * @param qdd_a The joint accelerations.
     * @param tau_a The joint torques.
     */
    void run(Eigen::VectorXd q_a, Eigen::VectorXd qd_a, Eigen::VectorXd qdd_a, Eigen::VectorXd tau_a, Eigen::VectorXd imu_data); 

    /**
     * @brief Retrieves the estimated state.
     * @return The estimated state as a matrix.
     */
    Eigen::MatrixXd get_state();

private: 
    /**
     * @brief Updates the internal state of the estimator.
     */
    void state_update(); 

    /**
     * @brief Constructs the robot model based on the provided model file.
     * @param model_path The path to the robot model file.
     */
    void robot_constructor(QString model_path); 

    /**
     * @brief Estimates the ground reaction forces.
     * @return True if the GRF estimation is successful, false otherwise.
     */
    bool grf_estimator(); 

    /**
     * @brief State machine that estimates the state of stepping
     * @return True if it is successful, false otherwise.
     */
    bool step_state_machine(); 

    /**
     * @brief Estimates the velocity of the floating base.
     * @return True if it is successful, false otherwise.
     */
    bool floating_base_estimator(); 

    VelKalmanFilter * accKalman; 
    StateData * state; ///< Pointer to the state data.
    Eigen::MatrixXd states; ///< Matrix to store and the estimated result states.

}; 

#endif // STATEESTIMATOR_H