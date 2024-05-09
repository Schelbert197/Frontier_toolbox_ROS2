#include <iosfwd>
#include <cmath>
#include <armadillo>
#include <unordered_set>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
    EKFSLAM::EKFSLAM() : Xi{m + 2 * n, arma::fill::zeros}, covariance{m + 2 * n, m + 2 * n, arma::fill::zeros}
    {
        initialize_covariance();
    }

    EKFSLAM::EKFSLAM(RobotConfiguration turtlebot_config): Xi{m + 2 * n, arma::fill::zeros}, covariance{m + 2 * n, m + 2 * n, arma::fill::zeros}
    {
        initialize_covariance();
        initialize_robot_config(turtlebot_config);
    }

    void EKFSLAM::initialize_covariance()
    {
        arma::mat cov_state, cov_obstacles, cov_zero1, cov_zero2;
        cov_state = arma::mat {m, m, arma::fill::zeros};
        cov_obstacles = arma::mat {2 * n, 2 * n, arma::fill::eye} *1000000.0;
        cov_zero1 = arma::mat {m, 2 * n, arma::fill::zeros};
        cov_zero2 = arma::mat {2 * n, m, arma::fill::zeros};
        covariance = arma::join_rows(arma::join_cols(cov_state, cov_zero2), arma::join_cols(cov_zero1, cov_obstacles));
    }

    void EKFSLAM::initialize_robot_config(RobotConfiguration init_pose)
    {
        q_t(0) = init_pose.theta;
        q_t(1) = init_pose.x;
        q_t(2) = init_pose.y;
        update_total_state_vector();
    }

    void EKFSLAM::update_total_state_vector()
    {
        // Populate with pose vector
        for (int pose_index = 0; pose_index < m; pose_index++) {
            Xi(pose_index) = q_t(pose_index);
        }
        // Populate with map vector
        for (int landmark_index = 0; landmark_index < n; landmark_index++) {
            Xi(m + (2 * landmark_index)) = mt(2 * landmark_index); // X coordinate of landmark
            Xi(m + (2 * landmark_index) + 1) = mt((2 * landmark_index) + 1); // Y coordinate of landmark
        }
    }

    void EKFSLAM::update_pose_and_map()
    {
        // Populate pose vector
        for (int pose_index = 0; pose_index < m; pose_index++)
        {
            q_t(pose_index) = Xi(pose_index);
        }
        // Populate map vector
        for (int landmark_index = 0; landmark_index < n; landmark_index++)
        {
            mt(2*landmark_index) = Xi(m + 2*landmark_index); // X coordinate of landmark
            mt(2*landmark_index + 1) = Xi(m + 2*landmark_index + 1); // Y coordinate of landmark
        }
    }

    void EKFSLAM::EKFSLAM_predict(Twist2D robot_twist)
    {
        if (!almost_equal(robot_twist.y , 0.0)) {
            throw std::logic_error("Twist cannot be accomplished without the wheels slipping!");
        } else { 
            ut(0) = robot_twist.omega;
            ut(1) = robot_twist.x;
            ut(2) = 0.0;
        }
        // Update state vector transition
        // If delta theta is 0
        if (almost_equal(ut(0), 0.0, 1.0e-5)) {
            Xi(1) = Xi(1) + (ut(1) * std::cos(Xi(0))) + wt;
            Xi(2) = Xi(2) + (ut(1) * std::sin(Xi(0))) + wt;
            Xi(0) = Xi(0) + 0.0 + wt;
        } else {
            Xi(1) = Xi(1) - ((ut(1) / ut(0)) * std::sin(Xi(0)) + (ut(1) / ut(0)) * std::sin((Xi(0) + ut(0)))) + wt;
            Xi(2) = Xi(2) + ((ut(1) / ut(0)) * std::cos(Xi(0)) - (ut(1) / ut(0)) * std::cos((Xi(0) + ut(0)))) + wt;
            Xi(0) = Xi(0) + ut(0) + wt;
        }

        // Top left of A matrix
        arma::mat pose_matrix(m, m, arma::fill::zeros);
        // If delta theta is 0
        if (almost_equal(ut(0), 0.0)) {     
            pose_matrix(1, 0) = -ut(1) * std::sin(Xi(0));
            pose_matrix(2, 0) = ut(1) * std::cos(Xi(0));
        } else {
            pose_matrix(1, 0) = -(ut(1) / ut(0)) * std::cos(Xi(0)) + (ut(1) / ut(0)) * std::cos(normalize_angle(Xi(0) + ut(0)));
            pose_matrix(2, 0) = -(ut(1) / ut(0)) * std::sin(Xi(0)) + (ut(1) / ut(0)) * std::sin(normalize_angle(Xi(0) + ut(0)));
        }
        // Fill in the rest of the A matrix with zeros
        arma::mat tr_0{m, 2 * n, arma::fill::zeros};
        arma::mat bl_0{2 * n, m, arma::fill::zeros};
        arma::mat br_0{2 * n, 2 * n, arma::fill::zeros};
        // Join matrices and add identity
        At = I + arma::join_vert(arma::join_horiz(pose_matrix, tr_0), arma::join_horiz(bl_0, br_0));

        // Create the Q_bar matrix
        arma::mat Q_bar = arma::join_vert(arma::join_horiz(Q, tr_0), arma::join_horiz(bl_0, br_0));

        // Update covariance matrix
        covariance = At * covariance * At.t(); // + Q_bar;
        // Check that the covariance is symmetric and positive semi-definite
        if (!(covariance.is_symmetric(1e-8))) {
            throw std::runtime_error("Error... Asymmetric covariance.");
        }
        arma::vec eigvals = arma::eig_sym(covariance);
        if (!(arma::all(eigvals >= 0))) {
            throw std::runtime_error("Error... Covariance is not positive semi-definite.");
        }

        // Update robot config using non linear g
        // Transform2D Tf_world_q{{Xi(1), Xi(2)}, Xi(0)};
        // auto Tf_world_rbtwist = integrate_twist(robot_twist);
        // Transform2D Tf_new_q = Tf_world_q * Tf_world_rbtwist;

        // Xi(0) = Tf_new_q.rotation();
        // Xi(1) = Tf_new_q.translation().x;
        // Xi(2) = Tf_new_q.translation().y;
        // update_total_state_vector();
        // update_pose_and_map();
    }

    void EKFSLAM::EKFSLAM_correct(double x, double y, size_t j)
    {
        // Range and bearing of landmark j
        double r_j = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        double phi_j = std::atan2(y, x);
        // Actual landmark measurement (not mean)
        z_j(0) = r_j;
        z_j(1) = phi_j;

        // Iterate through set to see if j is within the set
        // If landmark has not been seen before, add it to the map
        // Since our pose variables are predictions (distributions), our map is also a prediction. 
        if (seen_landmarks.find(j) == seen_landmarks.end()) {
            mt((j-1) * 2) = q_t(1) + r_j * std::cos(phi_j + q_t(0));
            mt(((j-1) * 2) + 1) = q_t(2) + r_j * std::sin(phi_j + q_t(0));
            seen_landmarks.insert(j);
            update_total_state_vector();
        }

        // Estimated relative x and y distances
        double del_x = mt(2*(j-1)) - q_t(1);
        double del_y = mt(2*(j-1)+1) - q_t(2);
        double d_t = std::pow(del_x, 2) + std::pow(del_y, 2);

        // Populate Hj matrix using the mini matrices
        arma::mat H_sub_1{2, 3, arma::fill::zeros};
        H_sub_1(0, 0) = 0.0;
        H_sub_1(1, 0) = -1.0;
        H_sub_1(0, 1) = -del_x/std::sqrt(d_t);
        H_sub_1(1, 1) = del_y/d_t;
        H_sub_1(0, 2) = -del_y/std::sqrt(d_t);
        H_sub_1(1, 2) = -del_x/d_t;
        arma::mat H_sub_2{2, 2*(j-1), arma::fill::zeros};
        arma::mat H_sub_3{2, 2, arma::fill::zeros};
        H_sub_3(0, 0) = del_x/std::sqrt(d_t);
        H_sub_3(1, 0) = -del_y/d_t;
        H_sub_3(0, 1) = del_y/std::sqrt(d_t);
        H_sub_3(1, 1) = del_x/d_t;
        arma::mat H_sub_4{2, 2 * (n - j), arma::fill::zeros};
        Hj = arma::join_horiz(arma::join_horiz(H_sub_1 ,H_sub_2), arma::join_horiz(H_sub_3 ,H_sub_4)); // NOT MULTIPLIED

        // Sensor noise matrix
        R = arma::mat{2, 2, arma::fill::eye} * R_noise;

        // Kalman gain from linearized model
        Ki = covariance * Hj.t() * ((Hj * covariance * Hj.t()) + R).i();

        // Posterior state update
        double r_j_hat = std::sqrt(d_t);
        double phi_j_hat = normalize_angle(atan2(del_y, del_x) - q_t(0));
        z_j_hat(0) = r_j_hat;
        z_j_hat(1) = phi_j_hat;
        Xi = Xi + (Ki * (z_j - z_j_hat));
        update_pose_and_map();

        // Posterior covariance
        covariance = (I - (Ki * Hj)) * covariance;
    }

    RobotConfiguration EKFSLAM::EKFSLAM_return_config() const
    {
        RobotConfiguration config{Xi(1), Xi(2), Xi(0)};
        return config;
    }

    arma::colvec EKFSLAM::return_Xi() const
    {
        return Xi;
    }

    arma::colvec EKFSLAM::return_map() const
    {
        return mt;
    }

    arma::mat EKFSLAM::return_state_matrix() const
    {
        return At;
    }

    arma::mat EKFSLAM::return_covariance() const
    {
        return covariance;
    }

    Twist2D EKFSLAM::return_twist_ut() const
    {
        return Twist2D{ut(0), ut(1), ut(2)};
    }

    /// \brief Return the actual measurement zj
    arma::mat EKFSLAM::return_actual_measurement() const
    {
        return z_j;
    }

    /// \brief Return the predicted measurement zj_hat
    arma::mat EKFSLAM::return_predicted_measurement() const
    {
        return z_j_hat;
    }

    /// \brief Return the sensor matrix Hj
    arma::mat EKFSLAM::return_sensor_matrix() const
    {
        return Hj;
    }
    

    // ### BEGIN CITATION 7 ###
    Twist2D EKFSLAM::differentiate_transform(const Transform2D & T_bB)
    {
        // Takes in T_bB and outputs Vb that results in T_bB
        // Check for pure translation
        if(almost_equal(T_bB.translation().y, 0.0) && almost_equal(T_bB.rotation(), 0.0)) {
            return Twist2D{0, T_bB.translation().x, 0};
        }
        else {
            // y = R (1 - cos(theta))
            double R = fabs(T_bB.translation().y / (1 - cos(T_bB.rotation())));
            // double R = fabs(T_bB.translation().y);
            // double R = fabs(-0.222);

            // return Twist2D{R, R, R};

            // Populate twist
            // Vb.x = R * theta_dot * cos(theta)
            return Twist2D{T_bB.rotation(), R * T_bB.rotation(), 0};
        }
        // Unanticipated error
        throw std::runtime_error("TRANSFORM DIFFERENTIATION FAILED.");
    } // ### END CITATION 7 ###

// #### Test stuff
// void EKFSLAM::EKFSLAM_predict(Twist2D robot_twist)
//     {
//         // Check for proper twist
//         if(!almost_equal(robot_twist.y , 0.0))
//         {
//             throw std::runtime_error("Improper twist for estimation!");
//         }

//         // Update twist
//         ut(0) = robot_twist.omega;
//         ut(1) = robot_twist.x;
//         ut(2) = 0.0;

//         // First we predict the covariance ˆΣ-_t using current A_t which is calculated using the previous state ξ_{t−1}, and current input u_t.
//         // ˆΣ¯_t = A_t ˆΣ_{t−1} A_t^{T} + Q-,

//         // Calculate A matrix.
//         arma::mat zeros_12{m, 2 * n, arma::fill::zeros};
//         arma::mat zeros_21{2 * n, m, arma::fill::zeros};
//         arma::mat zeros_22{2 * n, 2 * n, arma::fill::zeros};
//         arma::mat pose_state_matrix(m, m, arma::fill::zeros);

//         // Zero rotational velocity
//         if (almost_equal(ut(0), 0.0)) 
//         {     
//             pose_state_matrix(1, 0) = -ut(1) * sin(q_t(0));
//             pose_state_matrix(2, 0) = ut(1) * cos(q_t(0));
//         } 
//         // Non-zero rotational velocity
//         else 
//         {   
//             pose_state_matrix(1, 0) = -(ut(1) / ut(0)) * cos(q_t(0)) + (ut(1) / ut(0)) * cos(normalize_angle(q_t(0) + ut(0)));
//             pose_state_matrix(2, 0) = -(ut(1) / ut(0)) * sin(q_t(0)) + (ut(1) / ut(0)) * sin(normalize_angle(q_t(0) + ut(0)));
//         }

//         At = I +
//             arma::join_vert(
//             arma::join_horiz(pose_state_matrix, zeros_12),
//             arma::join_horiz(zeros_21, zeros_22));

//         // Calculate Q_bar matrix
//         arma::mat Q_bar =
//             arma::join_vert(
//             arma::join_horiz(Q, zeros_12), 
//             arma::join_horiz(zeros_21, zeros_22));
        
//         // Update covariance matrix
//         covariance = At * covariance * At.t() + Q_bar;
        
//         // Now we predict the mean pose ˆξ-_t using the current pose ξ_{t−1}
//         // ˆξ¯_t = g(ˆξ_{t−1}, u_t, 0)
        
//         // Predict mean pose nonlinearly
//         Transform2D pose_as_tf_Twb{Vector2D{q_t(1), q_t(2)}, q_t(0)}; // Original mean pose
//         Transform2D twist_as_tf_TbB = integrate_twist(robot_twist);   // Change in mean pose
//         Transform2D newpose_as_tf_TwB = pose_as_tf_Twb * twist_as_tf_TbB;  // Final mean pose

//         // Update pose and state
//         q_t(0) = newpose_as_tf_TwB.rotation();
//         q_t(1) = newpose_as_tf_TwB.translation().x;
//         q_t(2) = newpose_as_tf_TwB.translation().y;
//         update_total_state_vector();

//         // Check covariance matrix (symmetric and positive semi-definite)
//         // Check if symmetric
//         // if (!(covariance.is_symmetric(1e-8)))
//         // {
//         //     throw std::runtime_error("Covariance is Assymetric!!!");
//         // }
//         // // Check if positive semi-definite
//         // arma::vec eigvals = arma::eig_sym(covariance);
//         // if (!(arma::all(eigvals >= 0)))
//         // {
//         //     throw std::runtime_error("Covariance is not Positive Semi-Definite!!!");
//         // }

//     }

//     void EKFSLAM::EKFSLAM_correct(double x, double y, size_t j)
//     {
//         // Convert relative measurements to range-bearing
//         // r_j = (x^2 + y^2)^0.5
//         // ϕ_j = atan2(y, x)
//         double r_j = magnitude(Vector2D{x, y});
//         double phi_j = std::atan2(y, x);      // Normalize ?? TODO ??

//         // If landmark has not been seen before, add it to the map
//         // Since our pose variables are predictions (distributions), our map is also a prediction. 
//         if (seen_landmarks.find(j) == seen_landmarks.end()) 
//         {
//             // Initialize the landmark prediction as x and y predictions in map frame
//             mt(2 * (j-1)) = q_t(1) + r_j * cos(phi_j + q_t(0)); // ˆm_{x,j}
//             mt(2 * (j-1) + 1) = q_t(2) + r_j * sin(phi_j + q_t(0)); // ˆm_{y,j}
//             // Insert the new landmark index in the unordered_set
//             seen_landmarks.insert(j);
//             update_total_state_vector();
//         }
//         // Actual Measurement of that landmark. Not a distribution.
//         z_j(0) = r_j;
//         z_j(1) = phi_j;

//         // Relative predictions of landmark position, as cartesian coordinates
//         // δ_{x,j} = ˆm_{x,j} − ˆx_t
//         // δ_{y,j} = ˆm_{y,j} − ˆy_t
//         // d_j = δ_{x,j}^2 + δ_{y,j}^2
//         Vector2D delta_j{mt(2*(j-1)) - q_t(1), mt(2*(j-1)+1) - q_t(2)};
//         double d_j = std::pow(magnitude(delta_j), 2);

//         // Relative predictions of landmark position, as range-bearing
//         double r_j_hat = std::sqrt(d_j);
//         double phi_j_hat = normalize_angle(atan2(delta_j.y, delta_j.x) - q_t(0));
//         z_j_hat(0) = r_j_hat;
//         z_j_hat(1) = phi_j_hat;

//         // Calculate H matrix
//         arma::mat small_H_first{2, m, arma::fill::zeros}; // Dependence on pose
//         arma::mat zeros_2_first{2, 2 * (j-1), arma::fill::zeros}; // Dependence on landmarks having smaller indices
//         arma::mat small_H_second{2, 2, arma::fill::zeros}; // Dependence on sensed landmark
//         arma::mat zeros_2_second{2, 2 * n - 2 * j, arma::fill::zeros}; // Dependence on landmarks having larger indices

//         small_H_first(0, 0) = 0.0;
//         small_H_first(0, 1) = -delta_j.x / std::sqrt(d_j);
//         small_H_first(0, 2) = -delta_j.y / std::sqrt(d_j);
//         small_H_first(1, 0) = -1;
//         small_H_first(1, 1) = delta_j.y / d_j;
//         small_H_first(1, 2) = -delta_j.x / d_j;

//         small_H_second(0, 0) = delta_j.x / std::sqrt(d_j);
//         small_H_second(0, 1) = delta_j.y / std::sqrt(d_j);
//         small_H_second(1, 0) = -delta_j.y / d_j;
//         small_H_second(1, 1) = delta_j.x / d_j;

//         Hj = arma::join_horiz(arma::join_horiz(small_H_first, zeros_2_first), arma::join_horiz(small_H_second, zeros_2_second));

//         // Sensor noise matrix
//         R = arma::mat{2, 2, arma::fill::eye} * R_noise;
//         // Rj = R.submat(j, j, j + 1, j + 1);

//         // Kalman gain
//         // K_i = Σ¯_t H_{i}^{T} (H_i Σ¯_t H_{i}^{T} + R)^{-1}
//         Ki = covariance * Hj.t() * (Hj * covariance * Hj.t() + R).i();

//         // Update state to corrected prediction
        
//         // Subtract z_i and z_i_hat correcctly
//         arma::colvec z_j_diff{2, arma::fill::zeros};
//         z_j_diff(0) = z_j(0) - z_j_hat(0);
//         z_j_diff(1) = normalize_angle(z_j(1) - z_j_hat(1));

//         // ξ_t = ˆξ¯_t + K_i (z^i_t − ˆz^i_t)
//         Xi = Xi + Ki * (z_j_diff);    
//         update_pose_and_map();

//         // Update covariance
//         // Σ_t = (I − K_i H_i) Σ¯_t
//         covariance = (I - Ki * Hj) * covariance;

//         // Srikanth is super cool
//     }
}