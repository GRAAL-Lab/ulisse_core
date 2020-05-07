#include <libconfig.h++>
#include <ulisse_ctrl/configuration.h>
#include <ulisse_ctrl/ulisse_definitions.h>

void ConfigureTaskFromFile(std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj)
{
    for (auto& map : tasksMap) {
        map.second.task->ConfigFromFile(confObj);
    }
}

void ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, ulisse::TasksInfo>& tasksMap, libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& priorityLevels = root["priorityLevels"];

    std::vector<std::string> hierarchy;

    for (int i = 0; i < priorityLevels.getLength(); ++i) {

        const libconfig::Setting& priorityLevel = priorityLevels[i];
        std::string PLID;
        ctb::SetParam(priorityLevel, PLID, "name");

        hierarchy.push_back(PLID);

        //configure regularization data
        rml::RegularizationData regularizationData;
        ctb::SetParam(priorityLevel, regularizationData.params.lambda, "lambda");
        ctb::SetParam(priorityLevel, regularizationData.params.threshold, "threshold");

        actionManager->AddPriorityLevelWithRegularization(PLID, regularizationData);

        //add tasks to PL
        const libconfig::Setting& tasks = priorityLevel["tasks"];
        for (int i = 0; i < tasks.getLength(); ++i) {
            libconfig::Setting& task = tasks[i];

            std::cout << "Added task: " << task.c_str() << " to " << PLID << std::endl;

            std::unordered_map<std::string, ulisse::TasksInfo>::iterator it = tasksMap.find(task.c_str());

            actionManager->AddTaskToPriorityLevel(it->second.task, PLID);
        }
    }
    //Set the hierarchy
    actionManager->SetUnifiedHierarchy(hierarchy);
}

void ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& actions = root["actions"];

    std::vector<std::string> actionPL;

    for (int i = 0; i < actions.getLength(); ++i) {

        const libconfig::Setting& action = actions[i];
        std::string actionID;
        ctb::SetParam(action, actionID, "name");

        std::vector<std::string> actionPL;

        const libconfig::Setting& priorityLevels = action["levels"];
        for (int i = 0; i < priorityLevels.getLength(); ++i) {

            libconfig::Setting& priorityLevel = priorityLevels[i];

            actionPL.push_back(priorityLevel.c_str());
        }

        try {
            actionManager->AddAction(actionID, actionPL);
            std::cout << "Added: " << actionID << "action with PLs:" << std::endl;
            for (auto& pl : actionPL)
                std::cout << pl << std::endl;

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "Configuration Action Manager Exception:" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }
}

void ConfigureSatesFromFile(std::unordered_map<std::string, ulisse::states::GenericState&> statesMap, libconfig::Config& confObj)
{
    for (auto& mapLine : statesMap) {
        mapLine.second.ConfigureStateFromFile(confObj);
    }
}
