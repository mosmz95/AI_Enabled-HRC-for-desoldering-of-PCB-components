#ifndef DATA_OBJECT_HPP
#define DATA_OBJECT_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <utility>



class PcbComponent {
    public:
        /*
            This class stores the position of PCB component.
        */
        PcbComponent(const std::string &name,
                    const std::vector<double> &position,
                    const std::vector<double> &orientation,
                    const std::vector<double> &cobot_config,
                    double heat_time)
            : name_(name), tool_position_(position), tool_orientation_(orientation),
            cobot_config_(cobot_config), heat_duration_(heat_time), is_heated_(false) {}

        std::string componentName() const; 

        void gotHeated(); 

        void removalFailure(); 

        bool heatStatus() const; 
        const double& heatDuration() const;

        const std::vector<double>& toolPosition() const;

        const std::vector<double>& toolOrientation() const; 

        const std::vector<double>& cobotConfig() const; 

        std::vector<double> toolPose() const; 

        bool operator==(const PcbComponent &rhs) const;
    private:
        std::string name_;
        std::vector<double> tool_position_;
        std::vector<double> tool_orientation_;
        std::vector<double> cobot_config_;
        double heat_duration_;
        bool is_heated_;
};




class SafeRobotConfig {
    public:
        /*
            This class creates an object representing the robot safe configuration and its corresponding tool position in workspace.
        */
        SafeRobotConfig(const std::vector<double> &position,
                        const std::vector<double> &orientation,
                        const std::vector<double> &cobot_config)
            : tool_position_(position), tool_orientation_(orientation), cobot_config_(cobot_config) {}

        const std::vector<double>& safeCobotConfig() const;
        std::vector<double> toolPose() const; 

    private:
        std::vector<double> tool_position_;
        std::vector<double> tool_orientation_;
        std::vector<double> cobot_config_;
};



std::pair<std::vector<PcbComponent>, std::vector<SafeRobotConfig>> extract_data(const std::string& file_name);

#endif