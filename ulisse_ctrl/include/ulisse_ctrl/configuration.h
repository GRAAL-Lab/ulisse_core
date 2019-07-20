#ifndef ULISSE_CONFIGURATION_H
#define ULISSE_CONFIGURATION_H
#include <eigen3/Eigen/Dense>
#include <ikcl/ikcl.h>
#include <iostream>
#include <rml/RML.h>
#include <sstream>
#include <tpik/TPIKlib.h>
#include <vector>
#include "ulisse_ctrl/tasks/ActionTask.h"

bool GetPriorityLevelRegularizationDataFromFile // KCL
    (const std::string ID, const std::string confPath, rml::RegularizationData* regularizationData,
        const std::string propertyID);

bool InitializeUnifiedHierarchyAndActions(std::shared_ptr<tpik::ActionManager> actionManager,
    std::unordered_map<std::string, std::shared_ptr<tpik::Task> > taskIDMap, std::string confPath);

bool FindVectorConfFile(std::string property, Eigen::VectorXd& vector, const std::string confPath);

void ConfigureEqualityTaskFromFile(std::vector<std::shared_ptr<tpik::EqualityTask> > taskVector, const std::string confPath);

void ConfigureInequalityTaskFromFile(std::vector<std::shared_ptr<tpik::InequalityTask> > taskVector, const std::string confPath);

void ConfigureCartesianTaskFromFile(std::vector<std::shared_ptr<tpik::CartesianTask> > taskVector, const std::string confPath);

void ConfigureActionTaskFromFile(std::vector<std::shared_ptr<tpik::ActionTask> > taskVector, const std::string confPath);

bool GetVectorEigen(const std::string confPath, const std::string property, Eigen::VectorXd& out);

#endif // ULISSE_CONFIGURATION_H