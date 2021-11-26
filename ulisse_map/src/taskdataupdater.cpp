#include <QQmlContext>
#include <QMutex>

#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "taskdataupdater.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

TaskDataUpdater::TaskDataUpdater(QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , newIncomingAction_(false), actionLoaded_(false),
    taskDataUpdateInterval_(200)
{
}

TaskDataUpdater::TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , newIncomingAction_(false), actionLoaded_(false),
    taskDataUpdateInterval_(200)
{
    Init(engine);
}

TaskDataUpdater::~TaskDataUpdater()
{
    for(auto const& pl: tasksItemsMap_){
        for(auto const& taskQuickItem: pl.second){
            taskQuickItem->deleteLater();
        }
    }

    for(auto const& pl: qAsConst(plViewQuickItems_)){
        //qDebug() << "Deleting old PL View: " << pl->property("priorityID");
        pl->deleteLater();
    }

    slowTimer_->deleteLater();
    myTimer_->deleteLater();
}

void TaskDataUpdater::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));
    myTimer_->start(taskDataUpdateInterval_);

    /*slowTimer_ = new QTimer(this);
    QObject::connect(slowTimer_, SIGNAL(timeout()), this, SLOT(update_action_view()));
    int slowInterval = taskDataUpdateInterval_;
    slowTimer_->start(slowInterval);*/

    qtRootOjects_ = appEngine_->rootObjects();

    actionLabelObj_ = qtRootOjects_.first()->findChild<QObject*>("actionLabelObj");
    if (!actionLabelObj_) {
        qDebug() << "actionLabelObj_ Object NOT found!";
    }

    actionColumnViewObj_ = qtRootOjects_.first()->findChild<QObject*>("actionColumnViewObj");
    if (!actionColumnViewObj_) {
        qDebug() << "actionColumnViewObj_ Object NOT found!";
    }



    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // set the depth to the QoS profile
    //custom_qos_profile.depth = 7;
    tpikActionSub_ = this->create_subscription<ulisse_msgs::msg::TPIKAction>(ulisse_msgs::topicnames::tpik_action, 10,
        std::bind(&TaskDataUpdater::TPIKActionCB, this, _1) /*custom_qos_profile*/);

    absoluteAxisAlignmentSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment, 10,
        std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentCB, this, _1) /*custom_qos_profile*/);

    //tasksSubscribersMap_.insert({ulisse_msgs::topicnames::task_absolute_axis_alignment, absoluteAxisAlignmentSub_});

    //LoadAction();
}


void TaskDataUpdater::TPIKActionCB(const ulisse_msgs::msg::TPIKAction::SharedPtr msg)
{
    if(tpikActionMsg_.id != msg->id ){

        tpikActionMsg_ = *msg;

        //qDebug() << "New Incoming Action: " << tpikActionMsg_.id.c_str();
        newIncomingAction_ = true;
    }

}

void TaskDataUpdater::AbsoluteAxisAlignmentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
    //    ulisse_msgs::msg::TaskStatus aaa = *msg;
    //qDebug() << "task size: " << aaa.external_activation_function.size();
    //    qDebug() << "aaa.external_activation_function: " << aaa.external_activation_function;
    //    qDebug() << "aaa.id: " << aaa.id.c_str();
    //    qDebug() << "aaa.internal_activation_function: " << aaa.internal_activation_function;
    //    qDebug() << "aaa.enabled: " << aaa.enabled;
    //    //qDebug() << "aaa.in_current_action: " << aaa.in_current_action;
    //    qDebug() << "aaa.reference_rate: " << aaa.reference_rate;


}

void TaskDataUpdater::AbsoluteAxisAlignmentHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::AbsoluteAxisAlignmentSafetyCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::AngularPositionCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::CartesianDistanceCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::CartesianDistancePathFollowingCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::LinearHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::LinearVelocityCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}

void TaskDataUpdater::SafetyBoundariesCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    (void)msg;
}



/*QGeoCoordinate TaskDataUpdater::get_ulisse_pos()
{
    return q_ulisse_pos_;
}


QGeoCoordinate TaskDataUpdater::get_goal_pos()
{
    return q_goal_pos_;
}


double TaskDataUpdater::get_goal_distance()
{
    return q_goal_distance_;
}

double TaskDataUpdater::get_goal_heading()
{
    return q_goal_heading_deg_;
}

double TaskDataUpdater::get_accept_radius()
{
    return q_accept_radius_;
}*/

    void TaskDataUpdater::process_callbacks_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());

    if(newIncomingAction_){

        newIncomingAction_ = false;

        for(auto const& pl: tasksItemsMap_){
            for(auto const& taskQuickItem: pl.second){
                taskQuickItem->deleteLater();
            }
        }

        for(auto const& pl: qAsConst(plViewQuickItems_)){
            //qDebug() << "Deleting old PL View: " << pl->property("priorityID");
            pl->deleteLater();
        }

        LoadAction();
    }


    emit callbacks_processed();
}

/*void TaskDataUpdater::update_action_view()
{

    emit action_loaded();
}*/



void TaskDataUpdater::LoadAction()
{
    actionLabelObj_->setProperty("text", QVariant(QString(tpikActionMsg_.id.c_str()).replace("_", " ")));

    plViewQuickItems_.clear();
    tasksItemsMap_.clear();
    tasksColumnListObj_.clear();

    //qDebug() << "Recreating new PL Views...";

    for( const ulisse_msgs::msg::TPIKPriorityLevel &pl : tpikActionMsg_.priority_levels ){

        QVariant qPlName = QString(pl.id.c_str()).replace("_", " ");

        QQmlComponent plComponent(&engine_, QUrl("qrc:/qml/PriorityLevelData.qml"));
        plViewQuickItems_.push_back(qobject_cast<QQuickItem*>(plComponent.create()));
        QQmlEngine::setObjectOwnership(plViewQuickItems_.back(), QQmlEngine::CppOwnership);
        plViewQuickItems_.back()->setParentItem(qobject_cast<QQuickItem*>(actionColumnViewObj_));
        plViewQuickItems_.back()->setParent(actionColumnViewObj_);
        plViewQuickItems_.back()->setProperty("priorityID", qPlName);

        //qDebug() << "Created PL: " << plViewQuickItems_.back()->property("priorityID").toString();

        tasksItemsMap_.insert( { pl.id, QList<QQuickItem*>() } );

        tasksColumnListObj_.push_back(qtRootOjects_.first()->findChildren<QObject*>("taskDataColumnObj").back());

        //QStringList taskIDs;
        for (const auto &taskID : pl.tasks_id) {

            QVariant qTaskID = QString(taskID.c_str()).replace("_", " ");;

            QQmlComponent taskComponent(&engine_, QUrl("qrc:/qml/TaskData.qml"));
            tasksItemsMap_.at(pl.id).push_back((qobject_cast<QQuickItem*>(taskComponent.create())));
            QQmlEngine::setObjectOwnership(tasksItemsMap_.at(pl.id).back(), QQmlEngine::CppOwnership);
            tasksItemsMap_.at(pl.id).back()->setParentItem(qobject_cast<QQuickItem*>(tasksColumnListObj_.back()));
            tasksItemsMap_.at(pl.id).back()->setParent(tasksColumnListObj_.back());
            tasksItemsMap_.at(pl.id).back()->setProperty("taskName", qTaskID);

            //qDebug() << "Created Task: " << tasksItemsMap_.at(pl.id).back()->property("taskName").toString();
        }
    }

    actionLoaded_ = true;

    for(int i = 0; i < plViewQuickItems_.size(); i++ ){
    }

}

QVector<double> TaskDataUpdater::GenerateRandFloatVector(int size)
{
    QVector<double> randVect(size);

    for (int i = 0; i < randVect.size(); i++) {
        randVect[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }
    return randVect;
}

