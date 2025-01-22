#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include "ur5e_move/data_object.hpp"

std::vector<double> home() {
    return {1.5271029472351074, -1.5869223080077113, 2.188160244618551, -2.152525564233297, -1.6478660742389124, 3.8190863132476807};
}

int main() {
    // using json = nlohmann::json;
    
   char path[1024];
   std::string target_dir;
    if (realpath(__FILE__, path)) {
        std::string script_dir_ = path;
        std::string script_dir = script_dir_.substr(0, script_dir_.find_last_of("/\\"));;
        std::cout << "Script directory is: " << script_dir << std::endl;

        std::string parent_dir = script_dir.substr(0, script_dir.find_last_of("/\\"));
        std::cout << "Parent directory is: " << parent_dir << std::endl;

        target_dir = parent_dir + "/data_base/data_20240717_153332.json";
        std::cout << "Target file path: " << target_dir << std::endl;
    } else {
        std::cerr << "Error resolving the absolute path." << std::endl;
    }

    std::ifstream json_file(target_dir);
    if (!json_file.is_open()) {
        std::cerr << "Failed to open file: " << target_dir << std::endl;
        return 1;
    }

    // json raw_data;
    // json_file >> raw_data;
    // json_file.close();

    // std::cout << "Number of PCB componentsttttttttttttttttttttttttt: " << raw_data["pcb_components"].size() << std::endl;

    // std::vector<PcbComponent> list_of_components;
    // for (const auto &raw_component : raw_data["pcb_components"]) {
    //     PcbComponent temporary_component(
    //         raw_component["name"],
    //         raw_component["tool_position_xyz"].get<std::vector<double>>(),
    //         raw_component["tool_orientation_xyzw"].get<std::vector<double>>(),
    //         raw_component["cobot_JointSpace"].get<std::vector<double>>(),
    //         raw_component["Heat_duration"]);
    //     list_of_components.push_back(temporary_component);
    // }

    // if (raw_data.contains("Safety_configs")) {
    //     std::vector<SafeRobotConfig> list_of_safetyconfigs;
    //     for (const auto &raw_safety : raw_data["Safety_configs"]) {
    //         SafeRobotConfig temporary_safety(
    //             raw_safety["safety_tool_position_xyz"].get<std::vector<double>>(),
    //             raw_safety["safety_tool_orientation_xyzw"].get<std::vector<double>>(),
    //             raw_safety["safety_cobot_JointSpace"].get<std::vector<double>>());
    //         list_of_safetyconfigs.push_back(temporary_safety);
    //     }
    // }

    // for (const auto& key : raw_data.items()) {
    //     std::cout << key.key() << std::endl;
    // }

    // return 0;
    try {
        auto [components, safety_configs] = extract_data(target_dir);
        std::cout << "Number of PCB components: " << components.size() << std::endl;
        std::cout << "Number of Safety Configs: " << safety_configs.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
