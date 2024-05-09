// ### BEGIN Citation 7 ###
#include <sstream>
#include <cmath>
#include <memory>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <armadillo>

using turtlelib::normalize_angle;
using turtlelib::deg2rad;
using turtlelib::rad2deg;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::Transform2D;
using turtlelib::WheelPositions;
using turtlelib::RobotConfiguration;
using turtlelib::DiffDrive;
using turtlelib::EKFSLAM;
using turtlelib::wt;
using turtlelib::m;
using turtlelib::n;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

// Covariance matrix
arma::mat sigma_0_q = arma::zeros<arma::mat>(m, m); // We are absolutely sure about the initial pose
arma::mat sigma_0_m = arma::eye(2 * n, 2 * n) * 1e6; // Uncertainity in sensing, very high with no knowledge of obstacles
arma::mat zeros_12 = arma::zeros<arma::mat>(m, 2*n); // Zeros due to sensing and localization noise being independent
arma::mat zeros_21 = arma::zeros<arma::mat>(2*n, m); // Zeros due to sensing and localization noise being independent
arma::mat zeros_22 = arma::zeros<arma::mat>(2*n, 2*n); // 
arma::mat I = arma::eye(m + 2*n, m + 2*n); // 
arma::mat sigma_0 =
    arma::join_vert(
    arma::join_horiz(sigma_0_q, zeros_12), 
    arma::join_horiz(zeros_21, sigma_0_m));

// State Matrix
arma::mat A_0 = arma::zeros(m+2*n, m+2*n);

// Predictable process Noise
arma::mat Q{arma::mat{m,m,arma::fill::eye}*wt};
arma::mat Q_bar =
            arma::join_vert(
            arma::join_horiz(Q, zeros_12), 
            arma::join_horiz(zeros_21, zeros_22));

void ekf_check_pose(EKFSLAM subject, RobotConfiguration required_pose);
void ekf_check_map(EKFSLAM subject, arma::vec required_map);
void ekf_check_state_vector(EKFSLAM subject, arma::vec required_state_vector);
void ekf_check_covariance_matrix(EKFSLAM subject, arma::mat required_covariance_matrix);
void ekf_check_twist(EKFSLAM subject, Twist2D required_twist);
void ekf_check_state_matrix(EKFSLAM subject, arma::mat required_state_matrix);
void ekf_check_actual_measurement(EKFSLAM subject, arma::vec required_actual_measurement);
void ekf_check_EKFSLAM_predicted_measurement(EKFSLAM subject, arma::vec required_EKFSLAM_predicted_measurement);
void ekf_check_sensor_matrix(EKFSLAM subject, arma::mat required_sensor_matrix);

TEST_CASE( "Initialization works for EKFSLam", "[EKFSLAM()]") 
{
    EKFSLAM estimator;

    // Pose
    ekf_check_pose(estimator, RobotConfiguration{0.0, 0.0, 0.0});

    // Map
    ekf_check_map(estimator, arma::zeros<arma::vec>(2*n));

    // State vector
    ekf_check_state_vector(estimator, arma::zeros<arma::vec>(m + 2*n));

    // Covariance matrix
    ekf_check_covariance_matrix(estimator, sigma_0);

    // Twist
    ekf_check_twist(estimator, Twist2D{0.0, 0.0, 0.0});

    // State Matrix
    // ekf_check_state_matrix(estimator, A_0);

}

TEST_CASE( "Initialization with pose works for EKFSLam", "[EKFSLAM(RobotConfiguration)]") 
{
    RobotConfiguration pose{-69, 6.9, 4.20};
    EKFSLAM estimator(pose);

    // Pose
    ekf_check_pose(estimator, pose);

    // Map
    ekf_check_map(estimator, arma::zeros<arma::vec>(2*n));

    // State vector
    ekf_check_state_vector(estimator, arma::join_vert(arma::vec({pose.theta, pose.x, pose.y}), arma::zeros<arma::vec>(2*n)));

    // Covariance matrix
    ekf_check_covariance_matrix(estimator, sigma_0);

    // Twist
    ekf_check_twist(estimator, Twist2D{0.0, 0.0, 0.0});

    // State Matrix
    // ekf_check_state_matrix(estimator, A_0);

    
}

TEST_CASE( "Prediction works for EKFSLam", "[EKFSLAM_predict(Twist2D)]") 
{
    RobotConfiguration pose{6.9, 4.20, 0.0};

    std::unique_ptr<turtlelib::EKFSLAM> ekf_ptr_obj_;
    ekf_ptr_obj_ = std::make_unique<turtlelib::EKFSLAM>(pose);

    // EKFSLAM estimator(pose);

    // 1. Check for pure rotation
    Twist2D v1{3.1, 0, 0};
    ekf_ptr_obj_->EKFSLAM_predict(v1);

    // Pose
    ekf_check_pose((*ekf_ptr_obj_), RobotConfiguration{6.9, 4.20, 3.1});
    // Map
    ekf_check_map((*ekf_ptr_obj_), arma::zeros<arma::vec>(2*n));
    // State vector
    ekf_check_state_vector((*ekf_ptr_obj_), arma::join_vert(arma::vec({3.1, 6.9, 4.20}), arma::zeros<arma::vec>(2*n)));
    // Twist
    ekf_check_twist((*ekf_ptr_obj_), v1);
    // State matrix
    arma::mat A_1 = arma::eye(m+2*n, m+2*n);
    ekf_check_state_matrix((*ekf_ptr_obj_), A_1);
    // Covariance matrix
    arma::mat sigma_1 = A_1 * sigma_0 * A_1.t() + Q_bar;
    ekf_check_covariance_matrix((*ekf_ptr_obj_), sigma_1);

    // 2. Check for pure translation
    Twist2D v2{0, -6.9, 0};
    ekf_ptr_obj_->EKFSLAM_predict(v2);

    // Pose
    ekf_check_pose((*ekf_ptr_obj_), RobotConfiguration{13.7940325369, 3.9130934292, 3.1});
    // Map
    ekf_check_map((*ekf_ptr_obj_), arma::zeros<arma::vec>(2*n));
    // State vector
    ekf_check_state_vector((*ekf_ptr_obj_), arma::join_vert(arma::vec({3.1, 13.7940325369, 3.9130934292}), arma::zeros<arma::vec>(2*n)));
    // Twist
    ekf_check_twist((*ekf_ptr_obj_), v2);

    // State matrix
    arma::mat small_A_2 = arma::zeros<arma::mat>(m, m);
    small_A_2(1,0) = 6.9*std::sin(3.1);
    small_A_2(2,0) = -6.9*std::cos(3.1);
    arma::mat A_2 = I + arma::join_vert(
        arma::join_horiz(small_A_2, zeros_12), 
        arma::join_horiz(zeros_21, zeros_22));
    ekf_check_state_matrix((*ekf_ptr_obj_), A_2);

    // Covariance matrix
    arma::mat sigma_2 = A_2 * sigma_1 * A_2.t() + Q_bar;
    ekf_check_covariance_matrix((*ekf_ptr_obj_), sigma_2);

    // 3. Check for general twist
    Twist2D v3{0.42, 6.9, 0};
    ekf_ptr_obj_->EKFSLAM_predict(v3);

    // Pose
    ekf_check_pose((*ekf_ptr_obj_), RobotConfiguration{7.041534478, 2.7650493339, 3.52});
    // Map
    ekf_check_map((*ekf_ptr_obj_), arma::zeros<arma::vec>(2*n));
    // State vector
    ekf_check_state_vector((*ekf_ptr_obj_), arma::join_vert(arma::vec({3.52, 7.041534478, 2.7650493339}), arma::zeros<arma::vec>(2*n)));
    // Twist
    ekf_check_twist((*ekf_ptr_obj_), v3);

    // State matrix
    arma::mat small_A_3 = arma::zeros<arma::mat>(m, m);
    small_A_3(1,0) = (6.9/0.42)*(-std::cos(3.1)+std::cos(3.1+0.42));
    small_A_3(2,0) = (6.9/0.42)*(-std::sin(3.1)+std::sin(3.1+0.42));
    arma::mat A_3 = I + arma::join_vert(
        arma::join_horiz(small_A_3, zeros_12), 
        arma::join_horiz(zeros_21, zeros_22));
    ekf_check_state_matrix((*ekf_ptr_obj_), A_3);

    // Covariance matrix
    arma::mat sigma_3 = A_3 * sigma_2 * A_3.t() + Q_bar;
    ekf_check_covariance_matrix((*ekf_ptr_obj_), sigma_3);

    // 4. Check for improper twist (y-component)
    Twist2D v4{0.26, -6.9, -4.2};
    REQUIRE_THROWS(ekf_ptr_obj_->EKFSLAM_predict(v4));
}

TEST_CASE( "Correction works for EKFSLAM", "[EKFSLAM_correct(double, double, size_t)]") 
{
    RobotConfiguration pose{0.0, 0.0, 0.0};

    std::unique_ptr<turtlelib::EKFSLAM> ekf_ptr_obj_;
    ekf_ptr_obj_ = std::make_unique<turtlelib::EKFSLAM>(pose);

    // 1. Check for one-dimensional translation along x-axis
    Twist2D v1{0, -0.069, 0};
    ekf_ptr_obj_->EKFSLAM_predict(v1);
    ekf_ptr_obj_->EKFSLAM_correct(2.0 + 0.069, 0, 1);

    // Actual measurement
    arma::vec z_1 = arma::zeros<arma::vec>(2);
    z_1(0) = 2.0 + 0.069;
    z_1(1) = 0.0;
    ekf_check_actual_measurement((*ekf_ptr_obj_), z_1);

    // Predicted measurement
    arma::vec z_1_hat = arma::zeros<arma::vec>(2);
    z_1_hat(0) = 2.069;
    z_1_hat(1) = 0.0;
    ekf_check_EKFSLAM_predicted_measurement((*ekf_ptr_obj_), z_1_hat);

    // Sensor matrix
    arma::mat H_1 = arma::zeros<arma::mat>(2, m+2*n);

    H_1(0,0) = 0;
    H_1(0,1) = -1;
    H_1(0,2) = 0;
    H_1(1,0) = -1;
    H_1(1,1) = 0;
    H_1(1,2) = -1/2.069;

    H_1(0,3) = 1;
    H_1(0,4) = 0;
    H_1(1,3) = 0;
    H_1(1,4) = 1/2.069;
    
    ekf_check_sensor_matrix((*ekf_ptr_obj_), H_1);
}



void ekf_check_pose(EKFSLAM subject, RobotConfiguration required_pose)
{
    REQUIRE_THAT( subject.EKFSLAM_return_config().theta, WithinAbs(required_pose.theta,1.0e-6));
    REQUIRE_THAT( subject.EKFSLAM_return_config().x, WithinAbs(required_pose.x,1.0e-6));
    REQUIRE_THAT( subject.EKFSLAM_return_config().y, WithinAbs(required_pose.y,1.0e-6));
}

void ekf_check_map(EKFSLAM subject, arma::vec required_map)
{
    REQUIRE( arma::approx_equal(subject.return_map(), required_map, "reldiff", 1e-6));
}

void ekf_check_state_vector(EKFSLAM subject, arma::vec required_state_vector)
{
    REQUIRE( arma::approx_equal(subject.return_Xi(), required_state_vector, "reldiff", 1e-6));
}

void ekf_check_covariance_matrix(EKFSLAM subject, arma::mat required_covariance_matrix)
{
    // REQUIRE_THAT( subject.covariance_matrix()(1,0), WithinAbs(required_covariance_matrix(1,0),1.0e-6));
    REQUIRE( arma::approx_equal(subject.return_covariance(), required_covariance_matrix, "reldiff", 1e-6));
    REQUIRE( arma::approx_equal(subject.return_covariance().submat(0, m, m - 1, m + 2*n - 1), zeros_12, "reldiff", 1e-6));
    REQUIRE( arma::approx_equal(subject.return_covariance().submat(m, 0, m + 2*n - 1, m - 1), zeros_21, "reldiff", 1e-6));
}

void ekf_check_twist(EKFSLAM subject, Twist2D required_twist)
{
    REQUIRE_THAT( subject.return_twist_ut().omega, WithinAbs(required_twist.omega,1.0e-6));
    REQUIRE_THAT( subject.return_twist_ut().x, WithinAbs(required_twist.x,1.0e-6));
    REQUIRE_THAT( subject.return_twist_ut().y, WithinAbs(required_twist.y,1.0e-6));
}

void ekf_check_state_matrix(EKFSLAM subject, arma::mat required_state_matrix)
{
    REQUIRE( arma::approx_equal(subject.return_state_matrix(), required_state_matrix, "reldiff", 1e-6));
}

void ekf_check_actual_measurement(EKFSLAM subject, arma::vec required_actual_measurement)
{
    REQUIRE( arma::approx_equal(subject.return_actual_measurement(), required_actual_measurement, "reldiff", 1e-6));
}

void ekf_check_EKFSLAM_predicted_measurement(EKFSLAM subject, arma::vec required_EKFSLAM_predicted_measurement)
{
    // REQUIRE_THAT( subject.EKFSLAM_predicted_measurement()(0), WithinAbs(required_EKFSLAM_predicted_measurement(0),1.0e-6));
    REQUIRE( arma::approx_equal(subject.return_predicted_measurement(), required_EKFSLAM_predicted_measurement, "reldiff", 1e-6));
}

void ekf_check_sensor_matrix(EKFSLAM subject, arma::mat required_sensor_matrix)
{

    for(int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            // REQUIRE_THAT( subject.sensor_matrix()(j,i), WithinAbs(69.0,1.0e-6));
            REQUIRE_THAT( subject.return_sensor_matrix()(j,i), WithinAbs(required_sensor_matrix(j,i),1.0e-6));
        }
    }

    REQUIRE_THAT( subject.return_sensor_matrix().size(), WithinAbs(required_sensor_matrix.size(),1.0e-6));
    
    REQUIRE( arma::approx_equal(subject.return_sensor_matrix(), required_sensor_matrix, "absdiff", 1e-6));
}
// ### END Citation 7 ###