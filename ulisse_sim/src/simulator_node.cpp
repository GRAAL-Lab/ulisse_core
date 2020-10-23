#include "rclcpp/rclcpp.hpp"
#include "ulisse_sim/vehiclesimulator.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

using namespace std::chrono_literals;

bool LoadConfiguration(std::string fileName, std::shared_ptr<ulisse::SimulatorConfiguration>& config) noexcept(false);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simulator_node");

    ulisse::VehicleSimulator simulatedVehicle(node);

    std::string filename = "simulator_ulisse.conf";
    simulatedVehicle.config = std::make_shared<ulisse::SimulatorConfiguration>();

    if (!LoadConfiguration(filename, simulatedVehicle.config)) {
        std::cerr << "Simulator node: Failed to load config params from files" << std::endl;
        return -1;
    }

    simulatedVehicle.ulisseModel.params = simulatedVehicle.config->modelParams;

    std::cout << simulatedVehicle.config->modelParams << std::endl;

    rclcpp::WallRate loop_rate(simulatedVehicle.config->rate);

    while (rclcpp::ok()) {

        simulatedVehicle.ExecuteStep();
        simulatedVehicle.SimulateSensors();
        simulatedVehicle.PublishSensors();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

bool LoadConfiguration(std::string fileName, std::shared_ptr<ulisse::SimulatorConfiguration>& config) noexcept(false)
{
    //read conf file
    libconfig::Config confObj;

    //Inizialization
    std::string confPath = ament_index_cpp::get_package_share_directory("ulisse_sim").append("/conf/").append(fileName);

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return -1;
    }

    if (!config->ConfigureFromFile(confObj))
        return false;

    return true;
}
