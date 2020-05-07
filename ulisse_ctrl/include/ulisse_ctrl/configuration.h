#ifndef ULISSE_CONFIGURATION_H
#define ULISSE_CONFIGURATION_H

#include <ikcl/ikcl.h>
#include <ulisse_ctrl/ctrl_data_structs.hpp>
#include <ulisse_ctrl/states/genericstate.hpp>

void ConfigureTaskFromFile(std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
void ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
void ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj);
void ConfigureSatesFromFile(std::unordered_map<std::string, ulisse::states::GenericState &> statesMap, libconfig::Config& confObj);
#endif // ULISSE_CONFIGURATION_H
