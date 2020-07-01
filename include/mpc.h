#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>

using namespace std;
typedef CPPAD_TESTVECTOR(double) Dvector;

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////

/* Set the timestep length and duration */
const size_t N = 10;
const double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// NOTE: feel free to play around with this
// or do something completely different
const double ref_v = 90;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

//******************************************************************************
// Set the number of model variables (includes both states and inputs).
//
// For example: If the state is a 4 element vector, the actuators is a 2
// element vector and there are 10 timesteps. The number of variables is:
// 4 * 10 + 2 * 9
//******************************************************************************
const size_t n_vars = N * 6 + (N - 1) * 2;
// Set the number of constraints
const size_t n_constraints = N * 6;

////////////////////////////////////////////////////////////////////////////////
// MPC Class
////////////////////////////////////////////////////////////////////////////////

class MPC {
public:
  MPC();

  virtual ~MPC();

  double get_steering() { return m_steering; }

  double get_throttle() { return m_throttle; }

  vector<vector<double>> get_predicted_path() {
    return {m_predicted_xs, m_predicted_ys};
  }

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
  double m_steering;
  double m_throttle;

  Dvector m_vars; // where all the state and actuation variables will be stored
  Dvector m_vars_lowerbound; //lower limit for each corresponding variable in x
  Dvector m_vars_upperbound; //upper limit for each corresponding variable in x
  Dvector m_constraints_lowerbound; // value constraint for each corresponding constraint expression
  Dvector m_constraints_upperbound; // value constraint for each corresponding constraint expression

  vector<double> m_predicted_xs;
  vector<double> m_predicted_ys;
};

#endif /* MPC_H */
