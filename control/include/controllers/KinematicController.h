#pragma once

#include <memory>
#include <vector>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include "controllers/KDLRobot.h"

namespace controller {
class KinematicController {
public:
  KinematicController();
private:
  KDLRobot robot_;
};
}