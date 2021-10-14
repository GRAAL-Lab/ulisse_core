#ifndef ULISSE_CONFIGURATION_H
#define ULISSE_CONFIGURATION_H

#include <ikcl/ikcl.h>
#include <ulisse_ctrl/ctrl_data_structs.hpp>
#include <ulisse_ctrl/states/generic_state.hpp>

bool ConfigureTasksFromFile(std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
bool ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
bool ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj);
bool ConfigureSatesFromFile(std::unordered_map<std::string, std::shared_ptr<ulisse::states::GenericState>> statesMap, libconfig::Config& confObj);
#endif // ULISSE_CONFIGURATION_H
