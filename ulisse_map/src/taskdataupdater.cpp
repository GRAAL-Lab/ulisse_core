#include <QQmlContext>
#include <QMutex>

#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "taskdataupdater.hpp"
#include "ulisse_msgs/futils.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"


using std::placeholders::_1;

/*std::string Topic2TaskName(const std::string topicname){
    QString qtopicname(QString::fromStdString(topicname));
    qtopicname.replace("/ulisse/task/", "");
    return qtopicname.toStdString();
}*/

TaskDataUpdater::TaskDataUpdater(QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , newIncomingAction_(false)
    , taskDataUpdateInterval_(200)
{
}

TaskDataUpdater::TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , newIncomingAction_(false)
    , taskDataUpdateInterval_(200)
{
    Init(engine);
}

TaskDataUpdater::~TaskDataUpdater()
{
    for(auto const& taskQuickItem: tasksItemsMap_){
        taskQuickItem.second->deleteLater();
    }

    for(auto const& pl: qAsConst(plViewQuickItems_)){
        //qDebug() << "Deleting old PL View: " << pl->property("priorityID");
        pl->deleteLater();
    }

    //slowTimer_->deleteLater();
    myTimer_->deleteLater();
}

void TaskDataUpdater::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));

    //std::cout << "[TU] Timer interval = " << taskDataUpdateInterval_ << std::endl;
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

    RegisterSubscribers();

    tasksMessageMap_.insert( { ulisse::task::asvAbsoluteAxisAlignment, ulisse_msgs::msg::TaskStatus() } );
    tasksMessageMap_.insert( { ulisse::task::asvAbsoluteAxisAlignmentILOS, ulisse_msgs::msg::TaskStatus() } );//ILOS
    tasksMessageMap_.insert( { ulisse::task::asvAbsoluteAxisAlignmentALOS, ulisse_msgs::msg::TaskStatus() } );//ALOS
    tasksMessageMap_.insert( { ulisse::task::asvAbsoluteAxisAlignmentHold, ulisse_msgs::msg::TaskStatus() } );
    tasksMessageMap_.insert( { ulisse::task::asvAbsoluteAxisAlignmentSafety, ulisse_msgs::msg::TaskStatus() } );
    tasksMessageMap_.insert( { ulisse::task::asvAngularPosition, ulisse_msgs::msg::TaskStatus()});
//    tasksMessageMap_.insert( { ulisse::task::asvAngularPositionILOS, ulisse_msgs::msg::TaskStatus()});
    tasksMessageMap_.insert( { ulisse::task::asvCartesianDistance, ulisse_msgs::msg::TaskStatus()});
    tasksMessageMap_.insert( { ulisse::task::asvCartesianDistancePathFollowing, ulisse_msgs::msg::TaskStatus()});
    tasksMessageMap_.insert( { ulisse::task::asvLinearVelocity, ulisse_msgs::msg::TaskStatus()});
    tasksMessageMap_.insert( { ulisse::task::asvLinearVelocityHold, ulisse_msgs::msg::TaskStatus()});
    tasksMessageMap_.insert( { ulisse::task::asvLinearVelocityCurrentEst, ulisse_msgs::msg::TaskStatus()});//Current
    tasksMessageMap_.insert( { ulisse::task::asvSafetyBoundaries, ulisse_msgs::msg::TaskStatus()});
    tasksMessageMap_.insert( { ulisse::task::asvAbsoluteAxisAlignmentCurrentEst, ulisse_msgs::msg::TaskStatus() } );//Current

}

void TaskDataUpdater::RegisterSubscribers(){


    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // set the depth to the QoS profile
    //custom_qos_profile.depth = 7;

    tpikActionSub_ = this->create_subscription<ulisse_msgs::msg::TPIKAction>(ulisse_msgs::topicnames::tpik_action, 10,
                                                                             std::bind(&TaskDataUpdater::TPIKActionCB, this, _1));

    absoluteAxisAlignmentSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment, 10,
                                                                                        std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentCB, this, _1));

    absoluteAxisAlignmentILOSSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_ilos, 10,
        std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentILOSCB, this, _1));
        
    absoluteAxisAlignmentALOSSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_alos, 10,
                                                                                            std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentALOSCB, this, _1));

    absoluteAxisAlignmentHoldSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_hold, 10,
                                                                                            std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentHoldCB, this, _1));

    absoluteAxisAlignmentCurrentSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_current, 10,
        std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentCurrentCB, this, _1));

    absoluteAxisAlignmentSafetySub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_safety, 10,
                                                                                              std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentSafetyCB, this, _1));

    angularPositionSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_angular_position, 10,
        std::bind(&TaskDataUpdater::AngularPositionCB, this, _1));

    cartesianDistanceSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_cartesian_distance, 10,
                                                                                    std::bind(&TaskDataUpdater::CartesianDistanceCB, this, _1));

    cartesianDistancePathFollowingSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_cartesian_distance_path_follow, 10,
                                                                                                 std::bind(&TaskDataUpdater::CartesianDistancePathFollowingCB, this, _1));

    linearVelocitySub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity, 10,
                                                                                 std::bind(&TaskDataUpdater::LinearVelocityCB, this, _1));

    linearVelocityHoldSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity_hold, 10,
                                                                                     std::bind(&TaskDataUpdater::LinearVelocityHoldCB, this, _1));

    linearVelocityCurrentEstSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity_current, 10,
        std::bind(&TaskDataUpdater::LinearVelocityCurrentEstCB, this, _1));

    safetyBoundariesSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_safety_boundaries, 10,
                                                                                   std::bind(&TaskDataUpdater::SafetyBoundariesCB, this, _1));

}

void TaskDataUpdater::resetPublishersAndSubscribers()
{
    tpikActionSub_.reset();

    absoluteAxisAlignmentSub_         .reset();
    absoluteAxisAlignmentILOSSub_     .reset(); // ILOS
    absoluteAxisAlignmentALOSSub_     .reset(); // ALOS
    absoluteAxisAlignmentCurrentSub_  .reset(); // Current
    absoluteAxisAlignmentHoldSub_     .reset();
    absoluteAxisAlignmentSafetySub_   .reset();
    angularPositionSub_               .reset();
    cartesianDistanceSub_             .reset();
    cartesianDistancePathFollowingSub_.reset();
    linearVelocityHoldSub_            .reset();
    linearVelocityCurrentEstSub_      .reset(); // Current
    linearVelocitySub_                .reset();
    safetyBoundariesSub_              .reset();

    RegisterSubscribers();
}

void TaskDataUpdater::TPIKActionCB(const ulisse_msgs::msg::TPIKAction::SharedPtr msg)
{
    if(tpikActionMsg_.id != msg->id ){
        tpikActionMsg_ = *msg;
        newIncomingAction_ = true;
    }

}

void TaskDataUpdater::AbsoluteAxisAlignmentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvAbsoluteAxisAlignment) = *msg;
}

void TaskDataUpdater::AbsoluteAxisAlignmentILOSCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)// ILOS
{
    tasksMessageMap_.at(ulisse::task::asvAbsoluteAxisAlignmentILOS) = *msg;
}

void TaskDataUpdater::AbsoluteAxisAlignmentALOSCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)// ALOS
{
    tasksMessageMap_.at(ulisse::task::asvAbsoluteAxisAlignmentALOS) = *msg;
}

void TaskDataUpdater::AbsoluteAxisAlignmentHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvAbsoluteAxisAlignmentHold) = *msg;
}

void TaskDataUpdater::AbsoluteAxisAlignmentCurrentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvAbsoluteAxisAlignmentCurrentEst) = *msg;
}

void TaskDataUpdater::AbsoluteAxisAlignmentSafetyCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvAbsoluteAxisAlignmentSafety) = *msg;
}

void TaskDataUpdater::AngularPositionCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvAngularPosition) = *msg;
}

//void TaskDataUpdater::AngularPositionILOSCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
//{
//    tasksMessageMap_.at(ulisse::task::asvAngularPositionILOS) = *msg;
//}

void TaskDataUpdater::CartesianDistanceCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvCartesianDistance) = *msg;
}

void TaskDataUpdater::CartesianDistancePathFollowingCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvCartesianDistancePathFollowing) = *msg;
}

void TaskDataUpdater::LinearVelocityCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvLinearVelocity) = *msg;
}

void TaskDataUpdater::LinearVelocityHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvLinearVelocityHold) = *msg;
}

void TaskDataUpdater::LinearVelocityCurrentEstCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvLinearVelocityCurrentEst) = *msg;
}

void TaskDataUpdater::SafetyBoundariesCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    tasksMessageMap_.at(ulisse::task::asvSafetyBoundaries) = *msg;
}

void TaskDataUpdater::process_callbacks_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());

    if(newIncomingAction_){

        newIncomingAction_ = false;

        for(auto const& taskQuickItem: tasksItemsMap_){
            taskQuickItem.second->deleteLater();
        }

        for(auto const& pl: qAsConst(plViewQuickItems_)){
            //qDebug() << "Deleting old PL View: " << pl->property("priorityID");
            pl->deleteLater();
        }

        LoadAction();
    } else {
        UpdateView();
    }

    emit callbacks_processed();
}


void TaskDataUpdater::LoadAction()
{
    actionLabelObj_->setProperty("text", QVariant(QString::fromStdString(tpikActionMsg_.id).replace("_", " ")));

    plViewQuickItems_.clear();
    tasksItemsMap_.clear();
    tasksColumnListObj_.clear();

    //qDebug() << "Recreating new PL Views...";

    for( const ulisse_msgs::msg::TPIKPriorityLevel &pl : tpikActionMsg_.priority_levels ){


        QVariant qPlName = QString::fromStdString(pl.id).replace("_", " ");

        QQmlComponent plComponent(&engine_, QUrl("qrc:/qml/PriorityLevelData.qml"));
        plViewQuickItems_.push_back(qobject_cast<QQuickItem*>(plComponent.create()));
        QQmlEngine::setObjectOwnership(plViewQuickItems_.back(), QQmlEngine::CppOwnership);
        plViewQuickItems_.back()->setParentItem(qobject_cast<QQuickItem*>(actionColumnViewObj_));
        plViewQuickItems_.back()->setParent(actionColumnViewObj_);
        plViewQuickItems_.back()->setProperty("priorityID", qPlName);
        //qDebug() << "Created PL: " << plViewQuickItems_.back()->property("priorityID").toString();

        tasksColumnListObj_.push_back(qtRootOjects_.first()->findChildren<QObject*>("taskDataColumnObj").back());

        for (const auto &taskID : pl.tasks_id) {

            QVariant qTaskID = QString::fromStdString(taskID).replace("_", " ");

            QQmlComponent taskComponent(&engine_, QUrl("qrc:/qml/TaskData.qml"));
            tasksItemsMap_.insert( { taskID, qobject_cast<QQuickItem*>(taskComponent.create()) } );
            QQmlEngine::setObjectOwnership(tasksItemsMap_.at(taskID), QQmlEngine::CppOwnership);
            tasksItemsMap_.at(taskID)->setParentItem(qobject_cast<QQuickItem*>(tasksColumnListObj_.back()));
            tasksItemsMap_.at(taskID)->setParent(tasksColumnListObj_.back());
            tasksItemsMap_.at(taskID)->setProperty("taskName", qTaskID);
            //qDebug() << "Created Task: " << tasksItemsMap_.at(pl.id).back()->property("taskName").toString();
        }
    }
}

void TaskDataUpdater::UpdateView()
{
    for( const ulisse_msgs::msg::TPIKPriorityLevel &pl : tpikActionMsg_.priority_levels ){
        for (const auto &taskID : pl.tasks_id) {
            if ( tasksMessageMap_.count(taskID) == 0 ) {
                qDebug() << "Task [" << taskID.c_str() << "] not found in map!";
            } else {
                QVariant reference = QString::fromStdString(futils::STLVectorToString(tasksMessageMap_.at(taskID).reference_rate, ','));
                tasksItemsMap_.at(taskID)->setProperty("reference", reference);

                QVariant internalAct = QString::fromStdString(futils::STLVectorToString(tasksMessageMap_.at(taskID).internal_activation_function, ','));
                tasksItemsMap_.at(taskID)->setProperty("internalAct", internalAct);

                QVariant externalAct = QString::fromStdString(futils::STLVectorToString(tasksMessageMap_.at(taskID).external_activation_function, ','));
                tasksItemsMap_.at(taskID)->setProperty("externalAct", externalAct);

                QVariant enabled = tasksMessageMap_.at(taskID).enabled;
                tasksItemsMap_.at(taskID)->setProperty("enabled", enabled);
            }
        }
    }
}

