#ifndef ULISSE_CONFIGURATION_H
#define ULISSE_CONFIGURATION_H

#include <ikcl/ikcl.h>
#include <ulisse_ctrl/ctrl_data_structs.hpp>

bool GetPriorityLevelRegularizationDataFromFile // KCL
    (const std::string ID, const std::string confPath, rml::RegularizationData* regularizationData,
        const std::string propertyID);

bool InitializeUnifiedHierarchyAndActions(std::shared_ptr<tpik::ActionManager> actionManager,
    std::unordered_map<std::string, ulisse::TasksInfo> tasksMap, std::string confPath);

bool FindVectorConfFile(std::string property, Eigen::VectorXd& vector, const std::string confPath);

void ConfigureTaskFromFile(std::vector<std::shared_ptr<tpik::EqualityTask>> taskVector, const std::string confPath);

void ConfigureTaskFromFile(std::vector<std::shared_ptr<tpik::InequalityTask>> taskVector, const std::string confPath);

void ConfigureTaskFromFile(std::vector<std::shared_ptr<tpik::CartesianTask>> taskVector, const std::string confPath);

void ConfigureTaskFromFile(std::unordered_map<std::string, ulisse::TasksInfo> &tasksMap, libconfig::Config &confObj);

#endif // ULISSE_CONFIGURATION_H
