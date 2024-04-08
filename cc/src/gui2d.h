#pragma once

#include <vector>
#include <string>
#include <utility>
#include <memory>

#include "gnuplot-iostream.h"
#include "occupancy_grid2d.h"
#include "polygon.h"
#include "node_eigen.h"

namespace game_engine {

  class Gui2D {
    public:
      // Structure that encapsulates a command and data associated with that
      // command. Need to buffer a set of these structures to pass to gnuplot
      // all at once.
      struct CommandUnit {
        std::vector<std::pair<double, double>> data_;
        std::string command_;

        CommandUnit(const std::string command, 
                    const std::vector<std::pair<double, double>> data)
          : data_(data),
            command_(command) {}
      };

      bool LoadOccupancyGrid(const OccupancyGrid2D* occupancy_grid);
      bool LoadPath(const std::vector<std::shared_ptr<Node2D>>& path);
      bool Display();

    private:
      Gnuplot gp_;
      std::vector<CommandUnit> command_units_;

  };
}
