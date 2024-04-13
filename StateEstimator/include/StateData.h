#ifndef STATEDATA_H
#define STATEDATA_H

#include <Eigen/Dense>
#include <rbdl/rbdl.h>

class StateData
{
    public: 
        // State estimator input data
        Eigen::VectorXd imu_data; 
        Eigen::VectorXd imu_data_init; 
        int imu_init; 
        Eigen::VectorXd q_motor; 
        Eigen::VectorXd qd_motor; 
        Eigen::VectorXd qdd_motor; 
        Eigen::VectorXd tau_motor;

        // State Update data 
        RigidBodyDynamics::Model * robot_model; 
        Eigen::Vector3d body_T_offset; 
        Eigen::Vector3d left_foot_T_offset; 
        Eigen::Vector3d right_foot_T_offset; 

        // Model data
        std::vector<uint> id_body;
        std::vector<RigidBodyDynamics::Body *> robot_body;
        std::vector<RigidBodyDynamics::Joint *> robot_joint;
        Eigen::Matrix<double, Eigen::Dynamic, 1> q_lbound;
        Eigen::Matrix<double, Eigen::Dynamic, 1> q_ubound;
        Eigen::Matrix<double, Eigen::Dynamic, 1> qd_bound;
        Eigen::Matrix<double, Eigen::Dynamic, 1> tau_bound;
        Eigen::Vector3d gravity; 
        int ndof = 0;

        // Middle State data
        Eigen::VectorXd grf; 
        Eigen::VectorXd grf_pre; 
        int touch_index; 
        int touch_index_pre; 
        int foot_type; 
        double grf_lb{100}; 
        double grf_ub{0.8 * 480};

        Eigen::MatrixXd body_jaccobi; 
        Eigen::MatrixXd left_foot_jaccobi; 
        Eigen::MatrixXd right_foot_jaccobi; 
        Eigen::MatrixXd body_jaccobi_dot;
        Eigen::MatrixXd left_foot_jaccobi_dot;
        Eigen::MatrixXd right_foot_jaccobi_dot;

        // Torso Position
        Eigen::Vector3d posTorso; 
        Eigen::Vector3d velTorso; 
        Eigen::Vector3d velTorso_filt; 

        int n_dof;
        int dim; 

        // State estimator output data
        Eigen::MatrixXd x_a_body; 
        Eigen::MatrixXd x_a_left_foot; 
        Eigen::MatrixXd x_a_right_foot; 

        StateData();
        ~StateData(); 

}; 

#endif // STATEDATA_H