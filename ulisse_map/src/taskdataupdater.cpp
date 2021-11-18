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
    , taskDataUpdateInterval_(200)
{
}

TaskDataUpdater::TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , taskDataUpdateInterval_(200)
{
    Init(engine);
}

TaskDataUpdater::~TaskDataUpdater()
{
    /*for (std::shared_ptr<QQuickItem> plView : plViewObject_) {
        plView->deleteLater();
    }*/

    myTimer_->deleteLater();
}

void TaskDataUpdater::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));
    myTimer_->start(taskDataUpdateInterval_);

    slowTimer_ = new QTimer(this);
    QObject::connect(slowTimer_, SIGNAL(timeout()), this, SLOT(load_action_view()));
    int slowInterval = 1000;
    slowTimer_->start(slowInterval);

    qtRootOjects_ = appEngine_->rootObjects();
    actionViewObj_ = qtRootOjects_.first()->findChild<QObject*>("actionColumnViewObj");
    if (!actionViewObj_) {
        qDebug() << "actionViewObj Object NOT found!";
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


    tasksSubscribersMap_.insert({ulisse_msgs::topicnames::task_absolute_axis_alignment, absoluteAxisAlignmentSub_});



    LoadAction();
}


double TaskDataUpdater::RadiansToCompassDegrees(const double angle_rad)
{
    double angle_compass = angle_rad * 180.0 / M_PI;
    if (angle_compass < 0) {angle_compass += 360.0;}
    return angle_compass;
}

void TaskDataUpdater::TPIKActionCB(const ulisse_msgs::msg::TPIKAction::SharedPtr msg)
{
    if(tpikActionMsg_.id != msg->id ){

        tpikActionMsg_ = *msg;

        qDebug() << "New Incoming Action: " << tpikActionMsg_.id.c_str();

        /// TODO FIX OBJECT DELETION

        /*QMutex deleteMutex;

        for (QQuickItem* plView : plViewObject_) {
            deleteMutex.lock();
            metaObject()->invokeMethod(plView, "deleteLater", Qt::QueuedConnection);
            deleteMutex.unlock();
        }*/



        LoadAction();
    }

}

void TaskDataUpdater::AbsoluteAxisAlignmentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
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

}

void TaskDataUpdater::AbsoluteAxisAlignmentSafetyCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::AngularPositionCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::CartesianDistanceCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::CartesianDistancePathFollowingCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::LinearHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::LinearVelocityCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::SafetyBoundariesCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}


void TaskDataUpdater::copyToClipboard(QString newText)
{
    QClipboard* clipboard = QGuiApplication::clipboard();
    clipboard->setText(newText);
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

    emit callbacks_processed();
}

void TaskDataUpdater::load_action_view()
{
    //qDebug() << "pl_object->isEnabled(): " << pl_object->isEnabled();
    emit action_loaded();
}


void TaskDataUpdater::LoadAction()
{
    QMetaObject::invokeMethod(actionViewObj_, "clearActionView", Qt::QueuedConnection);
    //plViewObjects_.clear();

    auto actionLabelObj = qtRootOjects_.first()->findChild<QObject*>("actionLabelObj");
    actionLabelObj->setProperty("text", QVariant(tpikActionMsg_.id.c_str()));

    /*for(int i = 0; i < tpikActionMsg_.priority_levels.size(); i++ ){
        QMetaObject::invokeMethod(actionViewObj_, "generatePriorityLevel", Qt::QueuedConnection);
    }*/

    //auto plViewTempObjects_ = qtRootOjects_.first()->findChildren<QObject*>("actionColumnViewObj", Qt::FindDirectChildrenOnly);

    //for(int i = 0; i < plViewTempObjects_.size(); i++ ){
    //    qDebug() << "Found PL " << plViewTempObjects_.at(i)->objectName();
    //}

    /*for(int i = 0; i < tpikActionMsg_.priority_levels.size(); i++ ){
        QVariant plName = QString(tpikActionMsg_.priority_levels.at(i).id.c_str());
        QStringList taskIDs;
        for (const auto &taskID : tpikActionMsg_.priority_levels.at(i).tasks_id) {
            taskIDs << taskID.c_str();
        }
        QVariant tasksName(taskIDs);

        plViewObjects_.push_back(plViewTempObjects_.at(i));

        //plViewObjects_.at(i)->setParent(qtRootOjects_.first());
        plViewObjects_.at(i)->setProperty("priorityID", plName);
        plViewObjects_.at(i)->setProperty("taskIDs", tasksName);
    }*/
}

void TaskDataUpdater::LoadAction2()
{
    auto actionLabelObj = qtRootOjects_.first()->findChild<QObject*>("actionLabelObj");
    actionLabelObj->setProperty("text", QVariant(tpikActionMsg_.id.c_str()));

    for( const ulisse_msgs::msg::TPIKPriorityLevel &pl : tpikActionMsg_.priority_levels ){

        QVariant plName = QString(pl.id.c_str());
        QStringList taskIDs;
        for (const auto &taskID : pl.tasks_id) {
            taskIDs << taskID.c_str();
        }
        QVariant tasksName(taskIDs);

        QQmlComponent component1(&engine_, QUrl("qrc:/qml/PriorityLevelData.qml"));
        plViewQuickItems_.push_back(qobject_cast<QQuickItem*>(component1.create()));
        QQmlEngine::setObjectOwnership(plViewQuickItems_.back(), QQmlEngine::CppOwnership);
        plViewQuickItems_.back()->setParentItem(qobject_cast<QQuickItem*>(actionViewObj_));
        plViewQuickItems_.back()->setParent(qtRootOjects_.first());
        plViewQuickItems_.back()->setProperty("priorityID", plName);
        plViewQuickItems_.back()->setProperty("taskIDs", tasksName);
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

