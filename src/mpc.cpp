#include "mpc.h"

using CppAD::AD;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Used in IPOPT
 */
class FG_eval {
public:

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  void operator()(ADvector& fg, const ADvector& m_vars) {
    //**************************************************************************
    // `fg` - a vector of the cost constraints;
    // `m_vars` - a vector of variable values (state & actuators)
    //**************************************************************************

    // The cost is stored in the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    //**************************************************************************
    // Reference State Cost
    //   Define the cost related the reference state and anything you think
    //   may be beneficial. */
    //**************************************************************************

    // Weights for how "important" each cost is - can be tuned
    const int cte_cost_weight = 3000;
    const int epsi_cost_weight = 3000;
    const int v_cost_weight = 1;
    const int delta_cost_weight = 5;
    const int a_cost_weight = 5;
    const int delta_change_cost_weight = 200;
    const int a_change_cost_weight = 10;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += cte_cost_weight * CppAD::pow(m_vars[cte_start + t], 2);
      fg[0] += epsi_cost_weight * CppAD::pow(m_vars[epsi_start + t], 2);
      fg[0] += v_cost_weight * CppAD::pow(m_vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += delta_cost_weight * CppAD::pow(m_vars[delta_start + t], 2);
      fg[0] += a_cost_weight * CppAD::pow(m_vars[a_start + t], 2);
      // try adding penalty for speed + steer
      fg[0] += 200 * CppAD::pow(m_vars[delta_start + t] * m_vars[v_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += delta_change_cost_weight * CppAD::pow(m_vars[delta_start + t + 1] - m_vars[delta_start + t], 2);
      fg[0] += a_change_cost_weight * CppAD::pow(m_vars[a_start + t + 1] - m_vars[a_start + t], 2);
    }

    //**************************************************************************
    // Setup Model Constraints
    //**************************************************************************

    // Initial constraints
    fg[1 + x_start] = m_vars[x_start];
    fg[1 + y_start] = m_vars[y_start];
    fg[1 + psi_start] = m_vars[psi_start];
    fg[1 + v_start] = m_vars[v_start];
    fg[1 + cte_start] = m_vars[cte_start];
    fg[1 + epsi_start] = m_vars[epsi_start];

    // The rest of the constraints
    for (int t = 0; t < N-1; t++) {
      // The current state at time t.
      AD<double> x0 = m_vars[x_start + t];
      AD<double> y0 = m_vars[y_start + t];
      AD<double> psi0 = m_vars[psi_start + t];
      AD<double> v0 = m_vars[v_start + t];
      AD<double> cte0 = m_vars[cte_start + t];
      AD<double> epsi0 = m_vars[epsi_start + t];

      // Only consider the actuation at time t.
      AD<double> delta0 = m_vars[delta_start + t];
      AD<double> a0 = m_vars[a_start + t];

      // The next state at time t+1.
      AD<double> x1 = m_vars[x_start + t + 1];
      AD<double> y1 = m_vars[y_start + t + 1];
      AD<double> psi1 = m_vars[psi_start + t + 1];
      AD<double> v1 = m_vars[v_start + t + 1];
      AD<double> cte1 = m_vars[cte_start + t + 1];
      AD<double> epsi1 = m_vars[epsi_start + t + 1];

      AD<double> y0_des = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psi0_des = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0, 2));

      //************************************************************************
      // relationship of current state + actuations and next state
      // based on our kinematic model
      // considering latency
      //************************************************************************
      AD<double> x1_pred = (x0 + v0 * CppAD::cos(psi0) * dt);
      AD<double> y1_pred = (y0 + v0 * CppAD::sin(psi0) * dt);
      AD<double> psi1_pred = (psi0 - v0 * delta0 / Lf * dt);
      AD<double> v1_pred = (v0 + a0 * dt);
      AD<double> cte1_pred = ((y0_des - y0) + (v0 * CppAD::sin(epsi0) * dt));
      AD<double> epsi1_pred = ((psi0 - psi0_des) - v0 * delta0 / Lf * dt);

      // Setup the rest of the model constraints
      fg[1 + x_start + t + 1] = x1 - x1_pred;
      fg[1 + y_start + t + 1] = y1 - y1_pred;
      fg[1 + psi_start + t + 1] = psi1 - psi1_pred;
      fg[1 + v_start + t + 1] = v1 - v1_pred;
      fg[1 + cte_start + t + 1] = cte1 - cte1_pred;
      fg[1 + epsi_start + t + 1] = epsi1 - epsi1_pred;
    }
  }
};

/*******************************************************************************
 * MPC class definition implementation.
 ******************************************************************************/
MPC::MPC() {

  //**************************************************************
  //* SET INITIAL VALUES OF VARIABLES
  //**************************************************************
  m_vars.resize(n_vars);

  // all states except the VAR_START are set to zero
  // the aformentioned states will be initialized when solve() is called
  for (int i = 0; i < n_vars; i++) {
    m_vars[i] = 0.0;
  }

  //**************************************************************
  //* SET UPPER AND LOWER LIMITS OF VARIABLES
  //**************************************************************
  m_vars_lowerbound.resize(n_vars);
  m_vars_upperbound.resize(n_vars);

  /* Set lower and upper limits for variables. */

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    m_vars_lowerbound[i] = -1.0e19;
    m_vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    m_vars_lowerbound[i] = -0.436332;
    m_vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    m_vars_lowerbound[i] = -1.0;
    m_vars_upperbound[i] = 1.0;
  }

  //**************************************************************
  //* SET UPPER AND LOWER LIMITS OF CONSTRAINTS
  //**************************************************************
  m_constraints_lowerbound.resize(n_constraints);
  m_constraints_upperbound.resize(n_constraints);

  // the first constraint for each state veriable
  // refer to the initial state conditions
  // this will be initialized when solve() is called
  // the succeeding constraints refer to the relationship
  // between succeeding states based on our kinematic model of the system
  for (int i = 0; i < n_constraints; i++) {
    m_constraints_lowerbound[i] = 0;
    m_constraints_upperbound[i] = 0;
  }
}

MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  // Restore the initial state
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the initial variable values
  m_vars[x_start] = x;
  m_vars[y_start] = y;
  m_vars[psi_start] = psi;
  m_vars[v_start] = v;
  m_vars[cte_start] = cte;
  m_vars[epsi_start] = epsi;

  m_constraints_lowerbound[x_start] = x;
  m_constraints_lowerbound[y_start] = y;
  m_constraints_lowerbound[psi_start] = psi;
  m_constraints_lowerbound[v_start] = v;
  m_constraints_lowerbound[cte_start] = cte;
  m_constraints_lowerbound[epsi_start] = epsi;

  m_constraints_upperbound[x_start] = x;
  m_constraints_upperbound[y_start] = y;
  m_constraints_upperbound[psi_start] = psi;
  m_constraints_upperbound[v_start] = v;
  m_constraints_upperbound[cte_start] = cte;
  m_constraints_upperbound[epsi_start] = epsi;

  //**************************************************************
  // Solve as an optimization problem using IPOPT
  //**************************************************************

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options,
    m_vars,
    m_vars_lowerbound,
    m_vars_upperbound,
    m_constraints_lowerbound,
    m_constraints_upperbound,
    fg_eval,
    solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  //****************************************************************************
  // Return the first actuator values.
  // The variables can be accessed with `solution.x[i]`.
  //
  //   Steering must be divided by deg2rad(25) to normalize within [-1, 1].
  //   Multiplying by Lf takes into account vehicle's turning ability
  //****************************************************************************
  m_steering = solution.x[delta_start] / (deg2rad(25) * Lf);
  m_throttle = solution.x[a_start];

  m_predicted_xs = {};
  m_predicted_ys = {};

  for (int i = 1; i < N; ++i) {
    m_predicted_xs.push_back(solution.x[x_start + i]);
    m_predicted_ys.push_back(solution.x[y_start + i]);
  }
}
