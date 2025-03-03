#ifndef UA_UTILITIES_HPP
#define UA_UTILITIES_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "oal/obstacle.hpp"
#include "oal/helper_functions.hpp"
#include "oal/misc.hpp"

inline void SyncObsData(const Obstacle& old_obs, Obstacle& new_obs, std::chrono::milliseconds time_diff) {
    new_obs = old_obs;
    new_obs.pose.position = ComputePosition(new_obs, std::chrono::duration_cast<std::chrono::seconds>(time_diff));
}

inline auto generateRange = [](double start, double end, double step) {
    std::vector<double> result;
    for (double i = start; i <= end; i += step) {
        double num = std::round(i * 100) / 100;
        if (num != 0) result.push_back(num);
    }
    return result;
};

inline std::chrono::seconds ToChrono(const rclcpp::Time& time) {
    auto out = time.seconds();
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(out));
}



#endif //UA_UTILITIES_HPP