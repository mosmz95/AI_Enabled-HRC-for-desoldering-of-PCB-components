#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include "ur5e_move/data_object.hpp"



std::string PcbComponent::componentName() const { return name_; }

void PcbComponent::gotHeated() {
    is_heated_ = true;
}

void PcbComponent::removalFailure() {
    is_heated_ = false;
}

bool PcbComponent::heatStatus() const {
    return is_heated_;
}

const std::vector<double>& PcbComponent::toolPosition() const {
    return tool_position_;
}

const std::vector<double>& PcbComponent::toolOrientation() const {
    return tool_orientation_;
}

const std::vector<double>& PcbComponent::cobotConfig() const {
    return cobot_config_;
}

std::vector<double> PcbComponent::toolPose() const {
    std::vector<double> pose = tool_position_;
    pose.insert(pose.end(), tool_orientation_.begin(), tool_orientation_.end());
    return pose;
}





const std::vector<double>& SafeRobotConfig::safeCobotConfig() const {
    return cobot_config_;
}

std::vector<double> SafeRobotConfig::toolPose() const {
    std::vector<double> pose = tool_position_;
    pose.insert(pose.end(), tool_orientation_.begin(), tool_orientation_.end());
    return pose;
}



std::pair<std::vector<PcbComponent>, std::vector<SafeRobotConfig>> extract_data(const std::string& jsn_pth) {


    // Open the JSON file
    std::ifstream json_file(jsn_pth);
    if (!json_file.is_open()) {
        throw std::runtime_error("Failed to open file: " + jsn_pth);
    }

    // Parse the JSON file
    nlohmann::json raw_data;
    json_file >> raw_data;

    // Extract PCB components
    std::vector<PcbComponent> list_of_components;
    for (const auto& raw_component : raw_data["pcb_components"]) {
        PcbComponent temporary_component(
            raw_component["name"],
            raw_component["tool_position_xyz"].get<std::vector<double>>(),
            raw_component["tool_orientation_xyzw"].get<std::vector<double>>(),
            raw_component["cobot_JointSpace"].get<std::vector<double>>(),
            raw_component["Heat_duration"]
        );
        list_of_components.push_back(temporary_component);
    }

    // Extract Safety Configurations
    std::vector<SafeRobotConfig> list_of_safetyconfigs;
    if (raw_data.contains("Safety_configs")) {
        for (const auto& raw_safety : raw_data["Safety_configs"]) {
            SafeRobotConfig temporary_safety(
                raw_safety["safety_tool_position_xyz"].get<std::vector<double>>(),
                raw_safety["safety_tool_orientation_xyzw"].get<std::vector<double>>(),
                raw_safety["safety_cobot_JointSpace"].get<std::vector<double>>()
            );
            list_of_safetyconfigs.push_back(temporary_safety);
        }
    }

    return {list_of_components, list_of_safetyconfigs};
}