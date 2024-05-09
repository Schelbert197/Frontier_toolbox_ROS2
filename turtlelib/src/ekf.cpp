#include <iostream>
#include <armadillo>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
EKFSLAM::EKFSLAM()
: Xi{m + 2 * n, arma::fill::zeros}, covariance{m + 2 * n, m + 2 * n, arma::fill::zeros}
{
  initialize_covariance();
}

EKFSLAM::EKFSLAM(RobotConfiguration robot_config)
: Xi{m + 2 * n, arma::fill::zeros}, covariance{m + 2 * n, m + 2 * n, arma::fill::zeros}
{
  initialize_covariance();
  initialize_robot_config(robot_config);
}

void EKFSLAM::initialize_covariance()
{
  arma::mat cov_state, cov_obstacles, cov_zero1, cov_zero2;
  cov_state = arma::mat {m, m, arma::fill::zeros};
  cov_obstacles = arma::mat {2 * n, 2 * n, arma::fill::eye} *1e6;
  cov_zero1 = arma::mat {m, 2 * n, arma::fill::zeros};
  cov_zero2 = arma::mat {2 * n, m, arma::fill::zeros};
  covariance =
    arma::join_rows(
    arma::join_cols(cov_state, cov_zero2), arma::join_cols(
      cov_zero1,
      cov_obstacles));
}

void EKFSLAM::initialize_robot_config(RobotConfiguration robot_config)
{
  Xi(0) = robot_config.theta;
  Xi(1) = robot_config.x;
  Xi(2) = robot_config.y;
}

void EKFSLAM::EKFSLAM_predict(Twist2D twist)
{
  // ### BEGIN CITATION 8 ###
  // Calculate commanded twist
  ut(0) = normalize_angle(twist.omega - prev_twist.omega);
  ut(1) = twist.x - prev_twist.x;
  ut(2) = twist.y - prev_twist.y;
  // Set previous twist
  prev_twist = twist;

  Xi(0) = Xi(0) + ut(0);
  Xi(1) = Xi(1) + ut(1);
  Xi(2) = Xi(2) + ut(2);
  // ### END CITATION 8 ###

  // Calculate At matrix
  arma::mat zero_3_2n{m, 2 * n, arma::fill::zeros};
  arma::mat zero_2n_2n{2 * n, 2 * n, arma::fill::zeros};
  arma::mat pose_matrix(m, m, arma::fill::zeros);
  if (almost_equal(ut(0), 0.0)) { // Zero rotational velocity
    pose_matrix(1, 0) = -(ut(1)) * std::sin(Xi(0));
    pose_matrix(2, 0) = ut(1) * std::cos(Xi(0));
    At = I +
      arma::join_rows(
      arma::join_cols(pose_matrix, zero_3_2n.t()), arma::join_cols(
        zero_3_2n,
        zero_2n_2n));
  } else { // Non-zero rotational velocity
    pose_matrix(1, 0) = -(ut(1) / ut(0)) * std::cos(Xi(0)) + (ut(1) / ut(0)) * std::cos(
      Xi(0) + ut(0));
    pose_matrix(2, 0) = -(ut(1) / ut(0)) * std::sin(Xi(0)) + (ut(1) / ut(0)) * std::sin(
      Xi(0) + ut(0));
    At = I +
      arma::join_rows(
      arma::join_cols(pose_matrix, zero_3_2n.t()), arma::join_cols(zero_3_2n, zero_2n_2n));
  }

  Q_bar =
    arma::join_rows(arma::join_cols(Q, zero_3_2n.t()), arma::join_cols(zero_3_2n, zero_2n_2n));
  covariance = At * covariance * At.t() + Q_bar;
}

void EKFSLAM::EKFSLAM_correct(double x, double y, size_t j)
{
  // Convert relative measurements to range-bearing
  double r_j = magnitude(Vector2D{x, y});
  double phi_j = std::atan2(y, x);

  // If landmark has not been seen before, add it to the map
  // Since our pose variables are predictions (distributions), our map is also a prediction.
  if (seen_landmarks.find(j) == seen_landmarks.end()) {
    // Initialize the landmark prediction as x and y predictions in map frame
    Xi(m + 2 * (j - 1)) = Xi(1) + r_j * std::cos(phi_j + Xi(0));
    Xi(m + 2 * (j - 1) + 1) = Xi(2) + r_j * std::sin(phi_j + Xi(0));
    // Insert the new landmark index in the unordered_set
    seen_landmarks.insert(j);
  }
  // Actual Measurement of that landmark. Not a distribution.
  z_j(0) = r_j;
  z_j(1) = phi_j;

  // Relative predictions of landmark position, as cartesian coordinates
  Vector2D delta_j{Xi(m + 2 * (j - 1)) - Xi(1), Xi(m + 2 * (j - 1) + 1) - Xi(2)};
  double d_j = std::pow(magnitude(delta_j), 2);

  // Relative predictions of landmark position, as range-bearing
  double r_j_hat = std::sqrt(d_j);
  double phi_j_hat = normalize_angle(atan2(delta_j.y, delta_j.x) - Xi(0));
  z_j_hat(0) = r_j_hat;
  z_j_hat(1) = phi_j_hat;

  // Calculate H matrix
  arma::mat small_H_1{2, m, arma::fill::zeros};   // Dependence on pose
  arma::mat zeros_2_1{2, 2 * (j - 1), arma::fill::zeros}; // Dependence on landmarks having smaller indices
  arma::mat small_H_2{2, 2, arma::fill::zeros};   // Dependence on sensed landmark
  arma::mat zeros_2_2{2, 2 * n - 2 * j, arma::fill::zeros};   // Dependence on landmarks having larger indices
  // First matrix
  small_H_1(0, 0) = 0.0;
  small_H_1(0, 1) = -delta_j.x / std::sqrt(d_j);
  small_H_1(0, 2) = -delta_j.y / std::sqrt(d_j);
  small_H_1(1, 0) = -1;
  small_H_1(1, 1) = delta_j.y / d_j;
  small_H_1(1, 2) = -delta_j.x / d_j;
  // Second matrix
  small_H_2(0, 0) = delta_j.x / std::sqrt(d_j);
  small_H_2(0, 1) = delta_j.y / std::sqrt(d_j);
  small_H_2(1, 0) = -delta_j.y / d_j;
  small_H_2(1, 1) = delta_j.x / d_j;

  Hj =
    arma::join_horiz(
    arma::join_horiz(small_H_1, zeros_2_1), arma::join_horiz(
      small_H_2,
      zeros_2_2));

  // Sensor noise matrix
  R = arma::mat{2, 2, arma::fill::eye} *R_noise;

  // Kalman gain
  Ki = covariance * Hj.t() * (Hj * covariance * Hj.t() + R).i();

  // Update state to corrected prediction

  // Subtract z_i and z_i_hat correcctly
  arma::colvec z_i_diff{2, arma::fill::zeros};
  z_i_diff(0) = z_j(0) - z_j_hat(0);
  z_i_diff(1) = normalize_angle(z_j(1) - z_j_hat(1));

  Xi = Xi + Ki * (z_i_diff);

  // Update covariance
  covariance = (I - Ki * Hj) * covariance;
}

Circle EKFSLAM::fit_circle(std::vector<turtlelib::Vector2D> cluster)
{
  // circle fitting stuff.
  auto x_hat_centroid = 0.0;
  auto y_hat_centroid = 0.0;
  auto num_elements = 0.0;

  for (size_t i = 0; i < cluster.size(); i++) {
    x_hat_centroid += cluster.at(i).x;
    y_hat_centroid += cluster.at(i).y;
    num_elements += 1.0;
  }
  // Completes equations 1 and 2
  x_hat_centroid /= num_elements;
  y_hat_centroid /= num_elements;

  // init vars for steps 4
  auto x_i = 0.0; // auto better than double. ?
  auto y_i = 0.0;
  auto z_i = 0.0;
  auto z_bar = 0.0;
  auto n = static_cast<double>(cluster.size());

  // init step 5 data matrix
  arma::mat Z{};

  for (size_t i = 0; i < cluster.size(); i++) {
    // complete steps 3 and 5
    x_i = cluster.at(i).x - x_hat_centroid; // eqn (3)
    y_i = cluster.at(i).y - y_hat_centroid; // eqn (4)
    z_i = (x_i * x_i) + (y_i * y_i); // Step 3
    Z.insert_rows(i, arma::rowvec{z_i, x_i, y_i, 1}); // eqn (6)

    // for step 4
    z_bar += z_i;
  }

  // divide to get actual mean in step 4
  z_bar /= n;

  // Step 6 moment matrix
  arma::mat M = (1 / n) * Z.t() * Z;

  // Step 7 create the constraint matrix
  arma::mat H(4, 4, arma::fill::zeros);
  H(0, 0) = 8.0 * z_bar; // eqn (7)
  H(3, 0) = 2.0;
  H(0, 3) = 2.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  // Step 9 establish all vectors and do Singular Value Decomposition
  arma::mat U{};
  arma::vec s{};
  arma::mat V{};
  arma::svd(U, s, V, Z); // eqn (9)
  arma::colvec A{};

  if (s(3) < 10e-12) {
    // step 10
    A = V.col(3);
  } else {
    // Step 11 Constraint inverse to Q sigma_4 greater than 10e-12
    arma::mat Y = V * arma::diagmat(s) * trans(V);    // eqn (10)

    arma::mat Q = Y * H.i() * Y; // eqn 10 with H inv.
    // eig vec & val Q
    arma::cx_vec eigval{};
    arma::cx_mat eigvec{};
    arma::eig_gen(eigval, eigvec, Q);
    // Get real
    arma::mat real_eigval = arma::real(eigval);
    arma::mat real_eigvec = arma::real(eigvec);
    // Set A* = smallest eig value's vec
    auto min = 1000000.0;
    size_t min_index = 0;

    for (size_t i = 0; i < real_eigval.size(); i++) {
      if (real_eigval(i) < min && real_eigval(i) > 0.0) { // Find the smallest positive eigval
        min = real_eigval(i);
        min_index = i;
      }
    }
    // Extract eigvec that corresponds with the eigen value
    arma::vec A_star = real_eigvec.col(min_index);
    A = Y.i() * A_star;
  }

  // Step 12 Equation for circle eqn (11)
  double a = -A(1) / (2.0 * A(0)); // eqn (12)
  double b = -A(2) / (2.0 * A(0)); // eqn (13)
  double R = std::sqrt(((A(1) * A(1)) + (A(2) * A(2)) - (4.0 * A(0) * A(3))) / (4.0 * A(0) * A(0))); // eqn (14)

  // Step 13 Get centroid coordinates
  double cx = a + x_hat_centroid;
  double cy = b + y_hat_centroid;

  turtlelib::Circle myCircle{cx, cy, R};
  return myCircle;
}

size_t EKFSLAM::data_association(double x, double y)
{
  // Convert relative measurements to range bearing
  double r_j = std::sqrt(x * x + y * y);
  double phi_j = std::atan2(y, x);
  z_j(0) = r_j;
  z_j(1) = phi_j;

  // Set temp Xi with current state and 
  // initialize the landmark estimate x and y coordinates in Xi
  Xi_temp = Xi;
  Xi_temp(m + (2 * N) + 1) = Xi_temp(1) + (r_j * std::cos(phi_j + Xi_temp(0)));
  Xi_temp(m + (2 * N )+ 1 + 1) = Xi_temp(2) + (r_j * std::sin(phi_j + Xi_temp(0)));

  // Mahalanobis distance for each landmark
  std::vector<arma::mat> distances{};

  for (int k = 0; k < (N + 1); k++) {
    //Step 4.1.1
    // Estimate measurements for the relative distaces
    Vector2D rel_dist__est_j;
    rel_dist__est_j.x = Xi_temp(m + (2 * k)) - Xi_temp(1);
    rel_dist__est_j.y = Xi_temp(m + (2 * k) + 1) - Xi_temp(2);
    double d_j = rel_dist__est_j.x * rel_dist__est_j.x + rel_dist__est_j.y * rel_dist__est_j.y;

    // Calculate H matrix
    arma::mat zeros_1j(2, 2 * k);
    arma::mat zeros_1nj(2, 2 * n - 2 * (k + 1));
    arma::mat empty_2_3(2, 3);
    arma::mat empty_2_2(2, 2);
    // first matrix
    empty_2_3(1, 0) = -1;
    empty_2_3(0, 1) = -rel_dist__est_j.x / std::sqrt(d_j);
    empty_2_3(1, 1) = rel_dist__est_j.y / d_j;
    empty_2_3(0, 2) = -rel_dist__est_j.y / std::sqrt(d_j);
    empty_2_3(1, 2) = -rel_dist__est_j.x / d_j;
    // second matrix
    empty_2_2(0, 0) = rel_dist__est_j.x / std::sqrt(d_j);
    empty_2_2(1, 0) = -rel_dist__est_j.y / d_j;
    empty_2_2(0, 1) = rel_dist__est_j.y / std::sqrt(d_j);
    empty_2_2(1, 1) = rel_dist__est_j.x / d_j;

    // H matrix for landmark k
    arma::mat Hk =
      arma::join_rows(arma::join_rows(empty_2_3, zeros_1j), arma::join_rows(empty_2_2, zeros_1nj));

    // Step 4.1.2 Noise
    R = arma::mat{2 * n, 2 * n, arma::fill::eye} * R_noise;
    // R matrix and covariance for landmark k
    arma::mat R_k = R.submat(k, k, k + 1, k + 1);
    arma::mat covariance_k = Hk * covariance * Hk.t() + R_k;

    // Step 4.1.3
    double r_j_hat = std::sqrt(d_j);
    double phi_j_hat = normalize_angle(
      atan2(rel_dist__est_j.y, rel_dist__est_j.x) - Xi_temp(0));
    z_j_hat(0) = r_j_hat;
    z_j_hat(1) = phi_j_hat;

    // Step 4.1.4 Calculate Mahalanobis distance
    arma::mat dist_k = ((z_j - z_j_hat).t()) * (covariance_k.i()) * (z_j - z_j_hat);

    distances.push_back(dist_k);
  }

  // Step 4.2 & 4.3 Set distance threshold to distance N+1
  arma::mat distance_threshold = distances.at(distances.size() - 1);
  size_t index = N + 1;
  bool new_landmark = true;

  for (size_t i = 0; i < distances.size(); i++) {
    if (distances.at(i)(0) < distance_threshold(0)) {
      distance_threshold = distances.at(i);
      index = i;
      new_landmark = false;     // Not a new landmark
    }
  }

  // Step 4.4 increase N if new landmark
  if (new_landmark == true) { // If it is a new landmark increase N
    N++;
  
  // check if ell is equal to the thing that we just added
  // also need to ensure we don't cause index out of bounds error
  int int_index = static_cast<int>(index);
  if (int_index == N - 1) {
    } else {
      N -= 1;
      // clear out the obstacle
      Xi.at(3 + 2 * N) = 0;
      Xi.at(3 + 2 * N + 1) = 0;
    }
  }
  return index;
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

arma::mat EKFSLAM::return_actual_measurement() const
{
  return z_j;
}

arma::mat EKFSLAM::return_predicted_measurement() const
{
  return z_j_hat;
}

arma::mat EKFSLAM::return_sensor_matrix() const
{
  return Hj;
}
}
