#include "TaskEnvFactory.h"
#include "CartPole.h"
#include "Acrobot.h"
#include "CartCentering.h"
#include "Pendulum.h"
#include "MountainCar.h"
#include "MountainCarContinuous.h"
#include "RecursiveForecast.h"


//Gazebo Addition Begins
#include "TurtleBot4_Nav.h"

#include "MuJoco_Ant.h"
#include "MuJoco_Ant_Goal.h"
#include "MuJoco_Inverted_Pendulum.h"
#include "MuJoco_Inverted_Double_Pendulum.h"
#include "MuJoco_Half_Cheetah.h"
#include "MuJoco_Reacher.h"
#include "MuJoco_Hopper.h"
#include "MuJoco_Walker2D.h"
#include "MuJoco_Humanoid_Standup.h"

TaskEnv* TaskEnvFactory::createTask(const std::string& name, std::unordered_map<std::string, std::any>& params) {
    static const std::map<std::string, CreatorFunc> registry = {
        {"Cartpole", [](std::unordered_map<std::string, std::any>&) { return new CartPole(); }},
        {"Acrobot", [](std::unordered_map<std::string, std::any>&) { return new Acrobot(); }},
        {"CartCentering", [](std::unordered_map<std::string, std::any>&) { return new CartCentering(); }},
        {"Pendulum", [](std::unordered_map<std::string, std::any>&) { return new Pendulum(); }},
        {"MountainCar", [](std::unordered_map<std::string, std::any>&) { return new MountainCar(); }},
        {"MountainCarContinuous", [](std::unordered_map<std::string, std::any>&) { return new MountainCarContinuous(); }},
        {"Sunspots", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Sunspots"); }},
        {"Mackey", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Mackey"); }},
        {"Laser", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Laser"); }},
        {"Offset", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Offset"); }},
        {"Duration", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Duration"); }},
        {"Pitch", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Pitch"); }},
        {"PitchBach", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("PitchBach"); }},
        {"Bach", [](std::unordered_map<std::string, std::any>&) { return new RecursiveForecast("Bach"); }},


	{"TurtleBot4_Nav", [](std::unordered_map<std::string, std::any>& params) { return new TurtleBot4_Nav(params); }},

      
        {"MuJoco_Ant", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Ant(params); }},
        {"MuJoco_Ant_Goal", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Ant_Goal(params); }},
        {"MuJoco_Inverted_Pendulum", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Inverted_Pendulum(params); }},
        {"MuJoco_Inverted_Double_Pendulum", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Inverted_Double_Pendulum(params); }},
        {"MuJoco_Half_Cheetah", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Half_Cheetah(params); }},
        {"MuJoco_Reacher", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Reacher(params); }},
        {"MuJoco_Hopper", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Hopper(params); }},
        {"MuJoco_Walker2D", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Walker2D(params); }},
        {"MuJoco_Humanoid_Standup", [](std::unordered_map<std::string, std::any>& params) { return new MuJoco_Humanoid_Standup(params); }}
    };

    auto it = registry.find(name);
    if (it != registry.end()) {
        return it->second(params);
    }
    return nullptr;
}
