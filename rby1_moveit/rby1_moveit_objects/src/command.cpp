#include "rby1_moveit_objects/command.hpp"

#include <cstdlib>
#include <stdexcept>

namespace rby1_moveit_objects {

const char * const USAGE =
  "usage: scene {add,move,remove,clear,list,load} ...\n"
  "\n"
  "Add, move, remove, list and load MoveIt collision objects for RB-Y1.\n"
  "\n"
  "  scene add box      NAME --xyz X Y Z --size SX SY SZ [--rpy R P Y] [--frame base]\n"
  "  scene add sphere   NAME --xyz X Y Z --radius R [--frame base]\n"
  "  scene add cylinder NAME --xyz X Y Z --radius R --height H [--rpy R P Y] [--frame base]\n"
  "  scene move NAME --velocity VX VY VZ --time T [--rate 10]   # slide it, then leave it there\n"
  "  scene remove NAME [NAME ...]\n"
  "  scene clear\n"
  "  scene list\n"
  "  scene load FILE.yaml   # see config/example_scene.yaml\n"
  "\n"
  "Poses are in --frame (default base, every RB-Y1's root link), metres and radians.\n";

namespace {

double number(const std::string & text, const std::string & option) {
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (text.empty() || *end != '\0') {
    throw std::invalid_argument("argument " + option + ": invalid number: '" + text + "'");
  }
  return value;
}

// The `count` values after args[i] (the option), advancing i past them.
std::vector<double> values(const std::vector<std::string> & args, size_t & i, size_t count) {
  const std::string option = args[i];
  if (i + count >= args.size()) {
    throw std::invalid_argument("argument " + option + ": expected " + std::to_string(count) + " value(s)");
  }
  std::vector<double> result;
  for (size_t k = 1; k <= count; ++k) result.push_back(number(args[i + k], option));
  i += count;
  return result;
}

std::string text(const std::vector<std::string> & args, size_t & i) {
  if (i + 1 >= args.size()) throw std::invalid_argument("argument " + args[i] + ": expected a value");
  return args[++i];
}

}  // namespace

Command parse_command(const std::vector<std::string> & args) {
  Command command;
  if (args.empty()) throw std::invalid_argument("a command is required: add, move, remove, clear, list, load");
  command.name = args[0];
  if (command.name == "-h" || command.name == "--help" || command.name == "help") {
    command.name = "help";
    return command;
  }
  if (command.name == "add") {
    if (args.size() < 3) throw std::invalid_argument("add needs a type (box, sphere, cylinder) and a name");
    command.spec.type = args[1];
    command.spec.name = args[2];
    bool has_xyz = false;
    for (size_t i = 3; i < args.size(); ++i) {
      const std::string & option = args[i];
      if (option == "--xyz") {
        command.spec.xyz = values(args, i, 3);
        has_xyz = true;
      } else if (option == "--size") {
        command.spec.size = values(args, i, 3);
      } else if (option == "--rpy") {
        command.spec.rpy = values(args, i, 3);
      } else if (option == "--radius") {
        command.spec.radius = values(args, i, 1)[0];
      } else if (option == "--height") {
        command.spec.height = values(args, i, 1)[0];
      } else if (option == "--frame") {
        command.spec.frame = text(args, i);
      } else {
        throw std::invalid_argument("add: unknown argument '" + option + "'");
      }
    }
    if (!has_xyz) throw std::invalid_argument("add: the following arguments are required: --xyz");
    return command;
  }
  if (command.name == "move") {
    if (args.size() < 2) throw std::invalid_argument("move needs the name of an object");
    command.names = {args[1]};
    bool has_velocity = false, has_time = false;
    for (size_t i = 2; i < args.size(); ++i) {
      const std::string & option = args[i];
      if (option == "--velocity") {
        command.velocity = values(args, i, 3);
        has_velocity = true;
      } else if (option == "--time") {
        command.time = values(args, i, 1)[0];
        has_time = true;
      } else if (option == "--rate") {
        command.rate = values(args, i, 1)[0];
      } else {
        throw std::invalid_argument("move: unknown argument '" + option + "'");
      }
    }
    if (!has_velocity || !has_time) {
      throw std::invalid_argument("move: the following arguments are required: --velocity, --time");
    }
    return command;
  }
  if (command.name == "remove") {
    if (args.size() < 2) throw std::invalid_argument("remove needs at least one name");
    command.names.assign(args.begin() + 1, args.end());
    return command;
  }
  if (command.name == "clear" || command.name == "list") {
    if (args.size() > 1) throw std::invalid_argument(command.name + " takes no arguments");
    return command;
  }
  if (command.name == "load") {
    if (args.size() != 2) throw std::invalid_argument("load needs exactly one file");
    command.file = args[1];
    return command;
  }
  throw std::invalid_argument("unknown command '" + command.name +
                              "' (choose from add, move, remove, clear, list, load)");
}

}  // namespace rby1_moveit_objects
