// The `scene` command line, parsed without touching ROS.
#pragma once

#include <string>
#include <vector>

#include "rby1_moveit_objects/object_spec.hpp"

namespace rby1_moveit_objects {

struct Command {
  std::string name;                // add, move, remove, clear, list, load, help
  ObjectSpec spec;                 // add
  std::vector<std::string> names;  // remove; move uses names[0]
  std::vector<double> velocity;    // move
  double time = 0.0;               // move
  double rate = 10.0;              // move
  std::string file;                // load
};

extern const char * const USAGE;

// `args` without the program name; std::invalid_argument saying what is wrong.
Command parse_command(const std::vector<std::string> & args);

}  // namespace rby1_moveit_objects
