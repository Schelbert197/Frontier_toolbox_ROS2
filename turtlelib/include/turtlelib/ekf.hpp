#ifndef EKFSLAM_INCLUDE_GUARD_HPP
#define EKFSLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief Extended Kalman Filter (SLAM).

#include <iosfwd>
#include <cmath>
#include <armadillo>
#include <unordered_set>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    /// \brief Landmark x and y coordinates
    struct Landmark
    {
        /// \brief The x coordinate
        double x = 0.0;
        /// \brief The y coordinate
        double y = 0.0;
    };

    /// @brief Circle object with x,y centroid and radius
    struct Circle
    {
        /// \brief the x coordinate of the circle's centroid
        double x = 0.0;
        /// \brief the y coordinate of the circle's centroid
        double y = 0.0;
        /// \brief the radius of the circle
        double radius = 0.0;
    };

    /// \brief The size of robot state vector
    constexpr int m=3;
    /// \brief The maximum number of obstacles
    constexpr int n=10;
    /// \brief The process noise
    constexpr double wt = 0.001;
    /// \brief The noise constant on landmarks
    constexpr double R_noise = 0.01;

    /// @brief Class
    class EKFSLAM
    {
    private:
        /// \brief State of the robot at time t
        arma::colvec q_t{m,arma::fill::zeros};
        /// \brief State of the map at time t (landmark x,y pos)
        arma::colvec mt{2*n,arma::fill::zeros};
        /// \brief State vector of the total system at time t [q; m]
        arma::colvec Xi{};
        /// \brief Given twist at time t
        arma::colvec ut{m,arma::fill::zeros};
        /// \brief Previous twist time t-1
        Twist2D prev_twist{0.0,0.0,0.0};
        /// \brief Identity matrix
        arma::mat I{m+(2*n),m+(2*n),arma::fill::eye};
        /// \brief At matrix (aka g'())
        arma::mat At{m+(2*n),m+(2*n),arma::fill::zeros};
        /// \brief Process noise for the robot motion model
        arma::mat Q{arma::mat{m,m,arma::fill::eye}*wt};
        /// \brief Process noise for robot motion model; full zeros for size match
        arma::mat Q_bar{m+(2*n),m+(2*n),arma::fill::zeros};
        /// \brief Actual measurement with range and bearing
        arma::colvec z_j{2,arma::fill::zeros};
        /// \brief Theoretical measurement based on state estimate
        arma::colvec z_j_hat{2,arma::fill::zeros};
        /// \brief H matrix (range bearing measurement derivative h' w/ zeros)
        arma::mat Hj{};
        /// \brief Kalman gain
        arma::mat Ki{};
        /// \brief Noise identity matrix
        arma::mat R{2*n,2*n,arma::fill::eye};
        /// \brief Noise for landmark j
        arma::mat Rj{};
        /// \brief Covariance
        arma::mat covariance{};
        /// \brief Set of previously seen landmark ID's
        std::unordered_set<int> seen_landmarks{};
        /// \brief Temporary state vector of the total system at time t [q; m]
        arma::colvec Xi_temp{};
        /// \brief Data association
        int N = 0;

    public:
        /// \brief start at origin and default the uncertainty
        EKFSLAM();

        /// \brief set robot start config and default the uncertainty
        /// \param turtlebot_config - robot start configuration
        explicit EKFSLAM(RobotConfiguration turtlebot_config);

        /// \brief set the initial guess of the covariance matrix
        void initialize_covariance();

        /// \brief set the initial state of the robot
        /// \param turtlebot_config - robot start configuration
        void initialize_robot_config(RobotConfiguration turtlebot_config);

        /// \brief combines the q and m vecs to create total state vec update
        void update_total_state_vector();

        /// \brief updates the q and m vecs 
        void update_pose_and_map();

        /// \brief Predict/estimate the robot state and propagate the uncertainty
        /// \param twist - twist control at time t
        void EKFSLAM_predict(Twist2D twist);

        /// \brief Correction calculations
        /// \param x - landmark x-coordinate
        /// \param y - landmark y-coordinate
        /// \param j - landmark index j
        void EKFSLAM_correct(double x, double y, size_t j);

        /// \brief circle fitting algorithm
        /// \param cluster (std::vector<turtlelib::Vector2D>) pass one cluster in to fit a circle to
        /// \return radius and x,y coordinates of circle (turtlelib::Circle)
        Circle fit_circle(std::vector<turtlelib::Vector2D> cluster);

        /// \brief returns the robot configuration at t
        RobotConfiguration EKFSLAM_return_config() const;

        /// \brief returns the total state Xi = [q; m] at t 
        arma::colvec return_Xi() const;

        /// \brief returns the map mt at t 
        arma::colvec return_map() const;

        /// \brief returns the covariance sigma at t
        arma::mat return_covariance() const;

        /// \brief returns the At matrix (g')
        arma::mat return_state_matrix() const;

        /// \brief returns the twist of the robot at t
        Twist2D return_twist_ut() const;

        /// \brief Return the actual measurement zj
        arma::mat return_actual_measurement() const;

        /// \brief Return the predicted measurement zj_hat
        arma::mat return_predicted_measurement() const;

        /// \brief Return the sensor matrix Hj
        arma::mat return_sensor_matrix() const;

        /// \brief Calculates the twist from a Transform2D object
        /// \param T_bB - the transform to be derived
        /// \return the twist of the transform
        Twist2D differentiate_transform(const Transform2D & T_bB);

        /// \brief Data association
        size_t data_association(double x, double y);
    };

}

#endif