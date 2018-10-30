#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include <ulisse_ctrl/vehiclecontroller.hpp>

#include "rml/RML.h"

int main(int argc, char* argv[])
{
    //    std::cout << "Argv Test:" << std::endl;
    //    for (int i = 1; i < argc; i++) {
    //        std::cout << "argv[" << i << "]: " << std::string(argv[i]) << std::endl;
    //    }
    //    std::cout << std::endl;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("controller_node");



    int rate = 50;
    double sampleTime = 1.0 / rate;
    rclcpp::WallRate loop_rate(50);

    Eigen::TransfMatrix wTv;
    auto myModel = std::make_shared<rml::RobotModel>(wTv, "myVehicle");

    ulisse::VehicleController myVC(node, sampleTime);

    while (rclcpp::ok()) {

        myVC.Run();
        myVC.PublishControl();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
