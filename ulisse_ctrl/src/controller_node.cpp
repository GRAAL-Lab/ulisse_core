#include "rclcpp/rclcpp.hpp"

#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ulisse_ctrl/terminal_utils.hpp"
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

    int rate = 10;
    double sampleTime = 1.0 / rate;
    rclcpp::WallRate loop_rate(rate);

    Eigen::TransfMatrix wTv;
    auto myModel = std::make_shared<rml::RobotModel>(wTv, "myVehicle");

    ulisse::VehicleController myVC(node, sampleTime);

    ulisse::Spinner spinner(7);

    while (rclcpp::ok()) {

        spinner();

        myVC.Run();
        myVC.PublishControl();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
