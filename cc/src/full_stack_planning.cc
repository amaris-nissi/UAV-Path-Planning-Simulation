#include <cstdlib>
#include <vector>

#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"
#include "gui2d.h"

using namespace game_engine;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
  void writeToCSVfile(std::string name, const Eigen::MatrixXd matrix)
  {
      std::ofstream file(name.c_str());
      file << matrix.format(CSVFormat);
  };

int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file row1 col1 row2 col2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const std::shared_ptr<Node2D> end_ptr = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  /////////////////////////////////////////////////////////////////////////////
  // RUN A STAR
  // TODO: Run your A* implementation over the graph and nodes defined above.
  //       This section is intended to be more free-form. Using previous
  //       problems and examples, determine the correct commands to complete
  //       this problem. You may want to take advantage of some of the plotting
  //       and graphing utilities in previous problems to check your solution on
  //       the way.
  /////////////////////////////////////////////////////////////////////////////

  PathInfo path_info;
  AStar2D a_star2d;
  path_info = a_star2d.Run(graph, start_ptr, end_ptr);

  /////////////////////////////////////////////////////////////////////////////
  // RUN THE POLYNOMIAL PLANNER
  // TODO: Convert the A* solution to a problem the polynomial solver can
  //       solve. Solve the polynomial problem, sample the solution, figure out
  //       a way to export it to Matlab.
  ///////////////////////////////////////////////////////////////////////////// 

  std::vector<double> times = {}; // Define a vector for time, Time in seconds
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {}; // Define the node equality bonds

  for(int i=0; i<path_info.details.path_length; i++ ){
    times.push_back(i);
    if(i==0){
      node_equality_bounds.push_back(p4::NodeEqualityBound(0,0,0,path_info.path[0]->Data()[0])); // (dimension_index, node_idx, derivative_idx, value)
      node_equality_bounds.push_back(p4::NodeEqualityBound(1,0,0,path_info.path[0]->Data()[1]));
      node_equality_bounds.push_back(p4::NodeEqualityBound(0,0,1,0));
      node_equality_bounds.push_back(p4::NodeEqualityBound(1,0,1,0));
      node_equality_bounds.push_back(p4::NodeEqualityBound(0,0,2,0));
      node_equality_bounds.push_back(p4::NodeEqualityBound(1,0,2,0)); 
    } else {
      node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,0,path_info.path[i]->Data()[0]));
      node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,0,path_info.path[i]->Data()[1]));
    };
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
    sampler_options.frequency = 200;             // Number of samples per second
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

    if(sampler_options.derivative_order == 0)
      writeToCSVfile("a_star_plyn_pos.csv",samples);
    else if(sampler_options.derivative_order == 1)
      writeToCSVfile("a_star_plyn_vel.csv",samples);
    else
      writeToCSVfile("a_star_plyn_acc.csv",samples);
  }
  return EXIT_SUCCESS;
}