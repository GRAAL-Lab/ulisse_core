#ifndef ULISSE_CONFIGURATION_H
#define ULISSE_CONFIGURATION_H

#include <ikcl/ikcl.h>
#include <ulisse_ctrl/ctrl_data_structs.hpp>

void ConfigureTaskFromFile(std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
void ConfigurePriorityLevelFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
void ConfigureActionFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj);
void ConfigureSatesFromFile(std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj);
#endif // ULISSE_CONFIGURATION_H
