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

const double& PcbComponent::heatDuration() const {
    return heat_duration_;
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




bool PcbComponent::operator==(const PcbComponent &rhs) const{
    if (this->name_ == rhs.componentName()){
        return true;
    }else{
        return false;
    }

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


// this function returns a vector of PcbComponents from the ROS msg
std::vector<PcbComponent> createComponentsFromROSMsg(const std::vector<double> &location_x,
                                                     const std::vector<double> &location_y,
                                                     const std::vector<uint8_t> &component_class) {
    // Fixed values for orientation, cobot_config, and heat_time
    std::vector<double> empty_orientation; // Empty vector for orientation
    std::vector<double> empty_cobot_config; // Empty vector for cobot configuration
    double fixed_heat_time = 3.0; // Fixed heat time in seconds (example)

    // Predefined names for components based on their class IDs
    std::vector<std::string> component_names = {
        "Capacitor", "IC", "LED", "Resistor", "Battery", "Buzzer", "Clock",
        "Connector", "Diode", "Display", "Fuse", "Inductor", "Potentiometer",
        "Relay", "Switch", "Transistor"
    };

    // Vector to store PcbComponent instances
    std::vector<PcbComponent> components;

    // Check if input vectors have the same size
    if (location_x.size() != location_y.size() || location_x.size() != component_class.size()) {
        std::cerr << "Error: Input vectors must have the same size!" << std::endl;
        return components;
    }

    // Create components
    for (size_t i = 0; i < location_x.size(); ++i) {
        // Position of the component
        std::vector<double> position = {location_x[i], location_y[i]};

        // Get the name of the component based on its class ID
        std::string name = (component_class[i] < component_names.size()) ? component_names[component_class[i]]+ "_" + std::to_string(i) : "Unknown";

        // Create a PcbComponent instance and add it to the vector
        components.emplace_back(name, position, empty_orientation, empty_cobot_config, fixed_heat_time);
    }

    return components;
}