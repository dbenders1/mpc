
#include <mpc_modules/loader/module_handler.h>
#include <mpc_tools/printing.h>

// should be removed //////////////////////////////////////////////////
// #define reference_path
// #define reference_trajectory
// #define robust_tightening
// #define static_polyhedron_constraints
///////////////////////////////////////////////////////////////////////

// Objectives
#ifdef reference_path
#include <mpc_modules/objectives/reference_path.h>
#endif
#ifdef reference_trajectory
#include <mpc_modules/objectives/reference_trajectory.h>
#endif
#ifdef goal_oriented
#include <mpc_modules/objectives/goal_oriented.h>
#endif
// Constraints
#ifdef robust_tightening
#include <mpc_modules/constraints/robust_tightening.h>
#endif
#ifdef static_polyhedron_constraints
#include <mpc_modules/constraints/static_polyhedron_constraints.h>
#endif

ModuleHandler::ModuleHandler(ros::NodeHandle& nh, ConfigurationMPC* ptr_config,
                             SolverBase* ptr_solver,
                             DataSaverJson* ptr_data_saver_json)
    : modules_pointers_(ptr_solver->modules_names_.size()),
      ptr_solver_(ptr_solver),
      ptr_data_saver_json_(ptr_data_saver_json),
      modules_names_(ptr_solver->modules_names_.size()) {
  int index = 0;
  for (std::vector<std::string>::iterator iter =
           ptr_solver->modules_names_.begin();
       iter != ptr_solver->modules_names_.end(); iter++) {
#ifdef reference_path
    if (*iter == "ReferencePath") {
      modules_names_[index] = "ReferencePath";

      modules_pointers_[index].reset(
          new ReferencePath(modules_names_[index], nh, ptr_config, ptr_solver,
                            ptr_data_saver_json));
      ROS_WARN_STREAM("[Module handler] " << modules_names_[index]
                                          << " module loaded");
      index++;
      continue;
    }
#endif
#ifdef reference_trajectory
    if (*iter == "ReferenceTrajectory") {
      modules_names_[index] = "ReferenceTrajectory";

      modules_pointers_[index].reset(
          new ReferenceTrajectory(modules_names_[index], nh, ptr_config,
                                  ptr_solver, ptr_data_saver_json));
      ROS_WARN_STREAM("[Module handler] " << modules_names_[index]
                                          << " module loaded");
      index++;
      continue;
    }
#endif
#ifdef goal_oriented
    if (*iter == "GoalOriented") {
      modules_names_[index] = "GoalOriented";

      modules_pointers_[index].reset(new GoalOriented(modules_names_[index], nh,
                                                      ptr_config, ptr_solver,
                                                      ptr_data_saver_json));
      ROS_WARN_STREAM("[Module handler] " << modules_names_[index]
                                          << " module loaded");
      index++;
      continue;
    }
#endif
#ifdef robust_tightening
    if (*iter == "RobustTightening") {
      modules_names_[index] = "RobustTightening";

      modules_pointers_[index].reset(
          new RobustTightening(modules_names_[index], nh, ptr_config,
                               ptr_solver, ptr_data_saver_json, data_));
      ROS_WARN_STREAM("[Module handler] " << modules_names_[index]
                                          << " module loaded");
      index++;
      continue;
    }
#endif
#ifdef static_polyhedron_constraints
    if (*iter == "StaticPolyhedronConstraints") {
      modules_names_[index] = "StaticPolyhedronConstraints";

      modules_pointers_[index].reset(new StaticPolyhedronConstraints(
          modules_names_[index], nh, ptr_config, ptr_solver,
          ptr_data_saver_json, data_));
      ROS_WARN_STREAM("[Module handler] " << modules_names_[index]
                                          << " module loaded");
      index++;
      continue;
    }
#endif
    ROS_ERROR_STREAM(
        "[Module handler] Specified module from the solver isn't found (Not "
        "correctly defined in the CmakeList or not added to "
        "module_handler.cpp)! name: "
        << *iter);
  }
}

void ModuleHandler::onDataReceivedObjective(std::string name) {
  for (auto& module : modules_pointers_) {
    if (module->type_ == ModuleType::OBJECTIVE) {
      module->OnDataReceived(data_, name);
    };
  }
}

void ModuleHandler::onDataReceivedConstraint(std::string name) {
  for (auto& module : modules_pointers_) {
    if (module->type_ == ModuleType::CONSTRAINT) {
      module->OnDataReceived(data_, name);
    };
  }
}

void ModuleHandler::onDataReceived(std::string name) {
  for (auto& module : modules_pointers_) {
    module->OnDataReceived(data_, name);
  }
}

void ModuleHandler::updateModules() {
  for (auto& module : modules_pointers_) {
    module->Update(data_);
  }
}

void ModuleHandler::setParametersObjective() {
  for (auto& module : modules_pointers_) {
    if (module->type_ == ModuleType::OBJECTIVE) {
      module->SetParameters(data_);
    };
  }
}

void ModuleHandler::setParametersConstraint() {
  for (auto& module : modules_pointers_) {
    if (module->type_ == ModuleType::CONSTRAINT) {
      module->SetParameters(data_);
    };
  }
}

void ModuleHandler::exportDataJson(int count_since_start, int count_total) {
  for (auto& module : modules_pointers_) {
    module->ExportDataJson(count_since_start, count_total);
  }
}

void ModuleHandler::onReset() {
  for (auto& module : modules_pointers_) {
    module->OnReset();
  }
}

bool ModuleHandler::checkModulesReady() {
  bool ready_for_control_ = true;
  int i = 0;
  for (auto& module : modules_pointers_) {
    bool module_ready_ = module->ReadyForControl(data_);
    ready_for_control_ = ready_for_control_ && module_ready_;

    if (!module_ready_) {
      ROS_WARN_STREAM("[Module handler] Module " << modules_names_[i]
                                                 << " not ready for control!");
    };
    i++;
  }

  return ready_for_control_;
}

bool ModuleHandler::checkObjectiveReached() {
  bool objective_reached = true;
  for (auto& module : modules_pointers_) {
    objective_reached = objective_reached && module->ObjectiveReached(data_);
  }
  if (objective_reached) {
    ROS_WARN_STREAM("[Module handler] Objective reached!");
  };
  return objective_reached;
}

void ModuleHandler::tightenConstraints(std::string module_name,
                                       Eigen::VectorXd tightening_pred) {
  for (auto& module : modules_pointers_) {
    if (module->type_ == ModuleType::CONSTRAINT &&
        module->name_ == module_name) {
      module->TightenConstraints(tightening_pred);
    }
  }
}

void ModuleHandler::setConstraintsToRecAndVis(
    std::string module_name, const Eigen::MatrixXd& constraints,
    const Eigen::VectorXi& n_relevant_constraints) {
  for (auto& module : modules_pointers_) {
    if (module->type_ == ModuleType::CONSTRAINT &&
        module->name_ == module_name) {
      module->SetConstraintsToRecAndVis(constraints, n_relevant_constraints);
    }
  }
}

void ModuleHandler::publishData(const mpc_msgs::MpcHeader& mpc_header) {
  for (auto& module : modules_pointers_) {
    module->PublishData(mpc_header);
  }
}

void ModuleHandler::createVisualizationsModules() {
  for (auto& module : modules_pointers_) {
    module->CreateVisualizations();
  }
}

void ModuleHandler::publishVisualizationsModules() {
  for (auto& module : modules_pointers_) {
    module->PublishVisualizations();
  }
}

std::string ModuleHandler::getMethodName() {
  std::string s;
  for (std::vector<std::string>::const_iterator i =
           ptr_solver_->modules_names_.begin();
       i != ptr_solver_->modules_names_.end(); ++i)
    s += "_" + *i;
  return s;
}

void ModuleHandler::printRealTimeData() { data_.Print(); }
