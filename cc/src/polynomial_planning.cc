#include <cstdlib>
#include <vector>

#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"

///////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
void Example();
void DerivativeExperiments();
void ArrivalTimeExperiments();
void NumWaypointExperiments();

///////////////////////////////////////////////////////////////////
// MAIN FUNCTION
// TODO: UNCOMMENT THE FUNCTIONS YOU WANT TO RUN
///////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  Example();
  //DerivativeExperiments();
  //ArrivalTimeExperiments();
  //NumWaypointExperiments();

  return EXIT_SUCCESS;
}

// Example function that demonstrates how to use the polynomial solver. This
// example creates waypoints in a triangle: (0,0) -- (1,0) -- (1,1) -- (0,0)
void Example() {
  // Time in seconds
  const std::vector<double> times = {0,1,2,3};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize the 2nd order (acceleration)

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = true;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}

void DerivativeExperiments() {
  // Time in seconds
  // TODO: SET THE TIMES FOR THE WAYPOINTS
  const std::vector<double> times = {0,1,2,3,4};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A SQUARE TRAJECTORY
    ///////////////////////////////////////////////////////////////////
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,1),

    // The Fifth node constrains position
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 4;   // TODO: VARY THE DERIVATIVE ORDER

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}

void ArrivalTimeExperiments() {
  // Time in seconds
  // TODO: SET THE TIMES FOR THE WAYPOINTS
  const std::vector<double> times = {0,10,20,30,40};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A SQUARE TRAJECTORY
    ///////////////////////////////////////////////////////////////////
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,1),

    // The Fifth node constrains position
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 4;   // Minimize snap

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}

void NumWaypointExperiments() {
  // Time in seconds
  // TODO: SET THE TIMES FOR THE WAYPOINTS
  const std::vector<double> times = {0,1,2,3,4,5,6,7,8,
                                     9,10,11,12,13,14,15,16};
  /*
  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A CIRCLE TRAJECTORY
    ///////////////////////////////////////////////////////////////////
    
    p4::NodeEqualityBound(0,0,0,0.5),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,0.353),
    p4::NodeEqualityBound(1,1,0,0.353),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,0),
    p4::NodeEqualityBound(1,2,0,0.5),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,-0.353),
    p4::NodeEqualityBound(1,3,0,0.353),

    // The Fifth node constrains position
    p4::NodeEqualityBound(0,4,0,-0.5),
    p4::NodeEqualityBound(1,4,0,0),

    // The Sixth node constrains position
    p4::NodeEqualityBound(0,5,0,-0.353),
    p4::NodeEqualityBound(1,5,0,-0.353),

    // The Seventh node constrains position
    p4::NodeEqualityBound(0,6,0,0),
    p4::NodeEqualityBound(1,6,0,-0.5),

    // The Eighth node constrains position
    p4::NodeEqualityBound(0,7,0,0.353),
    p4::NodeEqualityBound(1,7,0,-0.353),

    // The ninth node constrains position
    p4::NodeEqualityBound(0,8,0,0.5),
    p4::NodeEqualityBound(1,8,0,0),
  };
  */
  ///*
  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A CIRCLE TRAJECTORY
    ///////////////////////////////////////////////////////////////////
    p4::NodeEqualityBound(0,0,0,0.5),
    p4::NodeEqualityBound(1,0,0,0.0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,0.461),
    p4::NodeEqualityBound(1,1,0,0.191),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,0.353),
    p4::NodeEqualityBound(1,2,0,0.353),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,0.191),
    p4::NodeEqualityBound(1,3,0,0.461),

    // The Fifth node constrains position
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0.5),

    // The Sixth node constrains position
    p4::NodeEqualityBound(0,5,0,-0.191),
    p4::NodeEqualityBound(1,5,0,0.461),

    // The Seventh node constrains position
    p4::NodeEqualityBound(0,6,0,-0.353),
    p4::NodeEqualityBound(1,6,0,0.353),

    // The Eighth node constrains position
    p4::NodeEqualityBound(0,7,0,-0.461),
    p4::NodeEqualityBound(1,7,0,0.191),

    // The ninth node constrains position
    p4::NodeEqualityBound(0,8,0,-0.5),
    p4::NodeEqualityBound(1,8,0,0),

    // The 10 node constrains position
    p4::NodeEqualityBound(0,9,0,-0.461),
    p4::NodeEqualityBound(1,9,0,-0.191),

    // The 11 node constrains position
    p4::NodeEqualityBound(0,10,0,-0.353),
    p4::NodeEqualityBound(1,10,0,-0.353),

    // The 12 node constrains position
    p4::NodeEqualityBound(0,11,0,-0.191),
    p4::NodeEqualityBound(1,11,0,-0.461),

    // The 13 node constrains position
    p4::NodeEqualityBound(0,12,0,0),
    p4::NodeEqualityBound(1,12,0,-0.5),

    // The 14 node constrains position
    p4::NodeEqualityBound(0,13,0,0.191),
    p4::NodeEqualityBound(1,13,0,-0.461),

    // The 15 node constrains position
    p4::NodeEqualityBound(0,14,0,0.353),
    p4::NodeEqualityBound(1,14,0,-0.353),

    // The 16 node constrains position
    p4::NodeEqualityBound(0,15,0,0.461),
    p4::NodeEqualityBound(1,15,0,-0.191),

    // The 17 node constrains position
    p4::NodeEqualityBound(0,16,0,0.5),
    p4::NodeEqualityBound(1,16,0,0),
  };
  //*/

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 4;   // Minimize snap

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}
