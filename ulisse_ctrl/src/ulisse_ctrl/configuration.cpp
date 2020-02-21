#include <libconfig.h++>
#include <ulisse_ctrl/configuration.h>
#include <ulisse_ctrl/ulisse_definitions.h>

bool FindVectorConfFile(std::string property, Eigen::VectorXd& vector, const std::string confPath)

{
    libconfig::Config confObj;
    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return false;
    }

    try {
        const libconfig::Setting& vectorSetting = confObj.lookup(property.c_str());
        vector.resize(vectorSetting.getLength());
        for (int i = 0; i < vectorSetting.getLength(); i++) {
            vector(i) = vectorSetting[i];
        }
        return true;

    } catch (libconfig::SettingException& e) {
        std::cerr << "Setting exception:" << std::endl;
        std::cerr << "path: " << e.getPath() << " what: " << e.what() << std::endl;
        return false;
    } catch (libconfig::ConfigException& e) {
        std::cerr << "Config exception:" << std::endl;
        std::cerr << "what: " << e.what() << std::endl;
        return false;
    }
}

bool InitializeUnifiedHierarchyAndActions(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, std::shared_ptr<tpik::Task>> taskIDMap, std::string confPath)
{
    // 1:1 mapping each priority level with the corresponing task
    for (size_t i = 0; i < ulisse::priorityLevelID::unified_hierarchy.size(); i++) {
        rml::RegularizationData regularization_data;
        std::string PLID = ulisse::priorityLevelID::unified_hierarchy.at(i);
        //if the task is in the file
        if (GetPriorityLevelRegularizationDataFromFile(PLID, confPath, &regularization_data, ulisse::priorityLevelParameter::priorityLevel)) {
            //Inizialized the PL with data
            actionManager->AddPriorityLevelWithRegularization(PLID, regularization_data);
            std::cout << "ADD TASK " << ulisse::task::unified_hierarchy.at(i) << std::endl;
            //Add the task to the proper level
            actionManager->AddTaskToPriorityLevel(taskIDMap.at(ulisse::task::unified_hierarchy.at(i)), PLID);
        }
    }

    actionManager->SetUnifiedHierarchy(ulisse::priorityLevelID::unified_hierarchy);

    try {
        actionManager->AddAction(ulisse::action::idle, ulisse::action::idlePriorityLevels);
        actionManager->AddAction(ulisse::action::speed_heading, ulisse::action::speed_headingPriorityLevels);
        actionManager->AddAction(ulisse::action::goTo, ulisse::action::goToPriorityLevels);
        actionManager->AddAction(ulisse::action::hold, ulisse::action::holdPriorityLevels);
        actionManager->AddAction(ulisse::action::navigate, ulisse::action::navigatePriorityLevels);

        return true;

    } catch (tpik::ExceptionWithHow& e) {
        std::cerr << "Configuration Action Manager Exception:" << std::endl;
        std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        return false;
    }
}

bool GetPriorityLevelRegularizationDataFromFile(const std::string ID, const std::string confPath, rml::RegularizationData* regularizationData, const std::string propertyID)
{
    libconfig::Config confObj;
    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError()
                  << std::endl;
        return false;
    }
    const std::string lambdaLookUp = propertyID + "." + ID + "." + ulisse::priorityLevelParameter::regularizationParameter + "." + ulisse::priorityLevelParameter::lambda;
    const std::string thresholdLookUp = propertyID + "." + ID + "." + ulisse::priorityLevelParameter::regularizationParameter + "." + ulisse::priorityLevelParameter::threshold;

    try {
        regularizationData->params.lambda = confObj.lookup(lambdaLookUp.c_str());
        regularizationData->params.threshold = confObj.lookup(thresholdLookUp.c_str());

    } catch (libconfig::SettingException& e) {
        std::cerr << "Setting exception:" << std::endl;
        std::cerr << "path: " << e.getPath() << " what: " << e.what() << std::endl;
        return false;
    } catch (libconfig::ConfigException& e) {
        std::cerr << "Config exception:" << std::endl;
        std::cerr << "what: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void ConfigureTaskFromFile(std::vector<std::shared_ptr<tpik::EqualityTask>> taskVector, const std::string confPath)
{
    for (auto& task : taskVector) {
        libconfig::Config confObj;
        try {
            confObj.readFile(confPath.c_str());
        } catch (libconfig::ParseException& e) {
            std::cerr << "Parse exception when reading:" << confPath << std::endl;
            std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        }

        try {

            std::string taskID = ulisse::task::task + "." + task->GetID();
            tpik::TaskParameter taskParameter;
            std::string lookUpGain = taskID + "." + ulisse::taskParameter::gain;
            std::string lookUpSaturation = taskID + "." + ulisse::taskParameter::saturation;
            std::string lookUpTaskEnable = taskID + "." + ulisse::taskParameter::taskEnable;
            taskParameter.gain = confObj.lookup(lookUpGain.c_str());
            taskParameter.saturation = confObj.lookup(lookUpSaturation.c_str());
            taskParameter.taskEnable = confObj.lookup(lookUpTaskEnable.c_str());
            task->SetTaskParameter(taskParameter);

        } catch (libconfig::SettingException& e) {
            std::cerr << "Setting exception:" << std::endl;
            std::cerr << "path: " << e.getPath() << " what: " << e.what() << std::endl;

        } catch (libconfig::ConfigException& e) {
            std::cerr << "Config exception:" << std::endl;
            std::cerr << "what: " << e.what() << std::endl;
        }
    }
}

void ConfigureTaskFromFile(std::vector<std::shared_ptr<tpik::InequalityTask>> taskVector, const std::string confPath)
{
    for (auto& task : taskVector) {
        libconfig::Config confObj;
        try {
            confObj.readFile(confPath.c_str());
        } catch (libconfig::ParseException& e) {
            std::cerr << "Parse exception when reading:" << confPath << std::endl;
            std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        }
        std::string taskID = ulisse::task::task + "." + task->GetID();
        tpik::TaskParameter taskParameter;
        std::string lookUpGain = taskID + "." + ulisse::taskParameter::gain;
        std::string lookUpSaturation = taskID + "." + ulisse::taskParameter::saturation;
        std::string lookUpTaskEnable = taskID + "." + ulisse::taskParameter::taskEnable;
        try {
            taskParameter.gain = confObj.lookup(lookUpGain.c_str());
            taskParameter.saturation = confObj.lookup(lookUpSaturation.c_str());
            taskParameter.taskEnable = confObj.lookup(lookUpTaskEnable.c_str());
            task->SetTaskParameter(taskParameter);

            if (task->GetBellShapeIncreasingUsed()) {
                tpik::BellShapedParameter bellShapeIncreasing;
                std::string lookUpBellShapeIncreasingMin = taskID + "." + ulisse::bellShapeParameter::increasingBellShape
                    + "." + ulisse::bellShapeParameter::xmin;
                std::string lookUpBellShapeIncreasingMax = taskID + "." + ulisse::bellShapeParameter::increasingBellShape
                    + "." + ulisse::bellShapeParameter::xmax;
                Eigen::VectorXd xminEigen;
                Eigen::VectorXd xmaxEigen;
                if (FindVectorConfFile(lookUpBellShapeIncreasingMin, xminEigen, confPath)
                    & FindVectorConfFile(lookUpBellShapeIncreasingMax, xmaxEigen, confPath)) {

                    bellShapeIncreasing.xmin = xminEigen;
                    bellShapeIncreasing.xmax = xmaxEigen;
                    task->SetIncreasingBellShapedParameter(bellShapeIncreasing);
                } else {
                    std::cerr << "Unable to find bell shaped parameter for task " + task->GetID() << std::endl;
                }
            }

            if (task->GetBellShapeDecreasingUsed()) {
                tpik::BellShapedParameter bellShapeDecreasing;
                std::string lookUpBellShapeDecreasingMin = taskID + "." + ulisse::bellShapeParameter::decreasingBellShape
                    + "." + ulisse::bellShapeParameter::xmin;
                std::string lookUpBellShapeDecreasingMax = taskID + "." + ulisse::bellShapeParameter::decreasingBellShape
                    + "." + ulisse::bellShapeParameter::xmax;
                Eigen::VectorXd xminEigen;
                Eigen::VectorXd xmaxEigen;
                if (FindVectorConfFile(lookUpBellShapeDecreasingMin, xminEigen, confPath)
                    & FindVectorConfFile(lookUpBellShapeDecreasingMax, xmaxEigen, confPath)) {
                    bellShapeDecreasing.xmin = xminEigen;
                    bellShapeDecreasing.xmax = xmaxEigen;
                    task->SetDecreasingBellShapedParameter(bellShapeDecreasing);
                } else {
                    std::cerr << "Unable to find bell shaped parameter for task " + task->GetID() << std::endl;
                }
            }

        } catch (libconfig::SettingException& e) {
            std::cerr << "Setting exception:" << std::endl;
            std::cerr << "path: " << e.getPath() << " what: " << e.what() << std::endl;
        } catch (libconfig::ConfigException& e) {
            std::cerr << "Config exception:" << std::endl;
            std::cerr << "what: " << e.what() << std::endl;
        }
    }
}

void ConfigureTaskFromFile(std::vector<std::shared_ptr<tpik::CartesianTask>> taskVector, const std::string confPath)
{
    for (auto& task : taskVector) {
        libconfig::Config confObj;
        try {
            confObj.readFile(confPath.c_str());
        } catch (libconfig::ParseException& e) {
            std::cerr << "Parse exception when reading:" << confPath << std::endl;
            std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        }
        std::string taskID = ulisse::task::task + "." + task->GetID();

        tpik::TaskParameter taskParameter;
        std::string lookUpGain = taskID + "." + ulisse::taskParameter::gain;
        std::string lookUpSaturation = taskID + "." + ulisse::taskParameter::saturation;
        std::string lookUpTaskEnable = taskID + "." + ulisse::taskParameter::taskEnable;

        try {
            taskParameter.gain = confObj.lookup(lookUpGain.c_str());
            taskParameter.saturation = confObj.lookup(lookUpSaturation.c_str());
            taskParameter.taskEnable = confObj.lookup(lookUpTaskEnable.c_str());
            task->SetTaskParameter(taskParameter);

            if (task->GetType() == tpik::CartesianTaskType::Equality) {

                std::string lookUpControlReference = taskID + "." + ulisse::taskParameter::controlReference;
                Eigen::VectorXd controlReference;
                if (FindVectorConfFile(lookUpControlReference, controlReference, confPath)) {

                    task->SetControlVectorReference(controlReference);
                }
            } else if (task->GetType() == tpik::CartesianTaskType::InequalityIncreasing) {
                tpik::BellShapedParameter bellShapeIncreasing;
                std::string lookUpBellShapeIncreasingMin = taskID + "." + ulisse::bellShapeParameter::increasingBellShape
                    + "." + ulisse::bellShapeParameter::xmin;
                std::string lookUpBellShapeIncreasingMax = taskID + "." + ulisse::bellShapeParameter::increasingBellShape
                    + "." + ulisse::bellShapeParameter::xmax;
                Eigen::VectorXd xminEigen;
                Eigen::VectorXd xmaxEigen;
                if (FindVectorConfFile(lookUpBellShapeIncreasingMin, xminEigen, confPath)
                    & FindVectorConfFile(lookUpBellShapeIncreasingMax, xmaxEigen, confPath)) {

                    bellShapeIncreasing.xmin = xminEigen;
                    bellShapeIncreasing.xmax = xmaxEigen;
                    task->SetBellShapedParameter(bellShapeIncreasing);
                } else {
                    std::cerr << "Unable to find bell shaped parameter for task " + task->GetID() << std::endl;
                }
            }

            else if (task->GetType() == tpik::CartesianTaskType::InequalityDecreasing) {
                tpik::BellShapedParameter bellShapeDecreasing;
                std::string lookUpBellShapeDecreasingMin = taskID + "." + ulisse::bellShapeParameter::decreasingBellShape
                    + "." + ulisse::bellShapeParameter::xmin;
                std::string lookUpBellShapeDecreasingMax = taskID + "." + ulisse::bellShapeParameter::decreasingBellShape
                    + "." + ulisse::bellShapeParameter::xmax;
                Eigen::VectorXd xminEigen;
                Eigen::VectorXd xmaxEigen;
                if (FindVectorConfFile(lookUpBellShapeDecreasingMin, xminEigen, confPath)
                    & FindVectorConfFile(lookUpBellShapeDecreasingMax, xmaxEigen, confPath)) {
                    bellShapeDecreasing.xmin = xminEigen;
                    bellShapeDecreasing.xmax = xmaxEigen;
                    task->SetBellShapedParameter(bellShapeDecreasing);
                } else {
                    std::cerr << "Unable to find bell shaped parameter for task " + task->GetID() << std::endl;
                }
            } else if (task->GetType() == tpik::CartesianTaskType::InequalityInBetween) {
                tpik::BellShapedParameter bellShapeDecreasing;
                std::string lookUpBellShapeDecreasingMin = taskID + "." + ulisse::bellShapeParameter::decreasingBellShape
                    + "." + ulisse::bellShapeParameter::xmin;
                std::string lookUpBellShapeDecreasingMax = taskID + "." + ulisse::bellShapeParameter::decreasingBellShape
                    + "." + ulisse::bellShapeParameter::xmax;
                Eigen::VectorXd xminEigenDecreasing;
                Eigen::VectorXd xmaxEigenDecreasing;
                tpik::BellShapedParameter bellShapeIncreasing;
                std::string lookUpBellShapeIncreasingMin = taskID + "." + ulisse::bellShapeParameter::increasingBellShape
                    + "." + ulisse::bellShapeParameter::xmin;
                std::string lookUpBellShapeIncreasingMax = taskID + "." + ulisse::bellShapeParameter::increasingBellShape
                    + "." + ulisse::bellShapeParameter::xmax;
                Eigen::VectorXd xminEigenIncreasing;
                Eigen::VectorXd xmaxEigenIncreasing;
                if (FindVectorConfFile(lookUpBellShapeDecreasingMin, xminEigenDecreasing, confPath)
                    & FindVectorConfFile(lookUpBellShapeDecreasingMax, xmaxEigenDecreasing, confPath)
                    & FindVectorConfFile(lookUpBellShapeIncreasingMin, xminEigenIncreasing, confPath)
                    & FindVectorConfFile(lookUpBellShapeIncreasingMax, xmaxEigenIncreasing, confPath)) {
                    bellShapeDecreasing.xmin = xminEigenDecreasing;
                    bellShapeDecreasing.xmax = xmaxEigenDecreasing;

                    bellShapeIncreasing.xmin = xminEigenIncreasing;
                    bellShapeIncreasing.xmax = xmaxEigenIncreasing;
                    task->SetBellShapedParameterInBetween(bellShapeIncreasing, bellShapeDecreasing);

                } else {
                    std::cerr << "Unable to find bell shaped parameter for task " + task->GetID() << std::endl;
                }
            }
        } catch (libconfig::SettingException& e) {
            std::cerr << "Setting exception:" << std::endl;
            std::cerr << "path: " << e.getPath() << " what: " << e.what() << std::endl;
        } catch (libconfig::ConfigException& e) {
            std::cerr << "Config exception:" << std::endl;
            std::cerr << "what: " << e.what() << std::endl;
        }
    }
}

bool GetVectorEigen(const std::string confPath, const std::string property, Eigen::VectorXd& out)
{
    libconfig::Config confObj;
    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError()
                  << std::endl;
        return false;
    }
    try {
        const libconfig::Setting& vectorSetting = confObj.lookup(property.c_str());
        out.resize(vectorSetting.getLength());
        for (int i = 0; i < vectorSetting.getLength(); i++) {
            out(i) = vectorSetting[i];
        }
        return true;

    } catch (libconfig::SettingException& e) {
        std::cerr << "Setting exception:" << std::endl;
        std::cerr << "path: " << e.getPath() << " what: " << e.what() << std::endl;
        return false;
    } catch (libconfig::ConfigException& e) {
        std::cerr << "Config exception:" << std::endl;
        std::cerr << "what: " << e.what() << std::endl;
        return false;
    }
}
