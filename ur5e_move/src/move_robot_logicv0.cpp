#include "rclcpp/rclcpp.hpp"
#include "ur5e_move/moverobotclass.hpp"
#include "ur5e_move/heatsequencelogic.hpp"

class MoveitDummyNode : public rclcpp::Node{

    public:
        MoveitDummyNode(rclcpp::NodeOptions& node_opt):Node("dummy_move_group",node_opt){
            RCLCPP_INFO(this->get_logger(),"Dummy node has been initializesd");
        }
};


int main(int argc, char **argv){

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


    try {
        auto [components, safety_configs] = extract_data(target_dir);
        std::cout << "Number of PCB components: " << components.size() << std::endl;
        std::cout << "Number of Safety Configs: " << safety_configs.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }



    auto [components, safety_configs] = extract_data(target_dir);

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<MoveitDummyNode> node_movegroup = std::make_shared<MoveitDummyNode>(node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_movegroup);
    std::thread([&executor]() { executor.spin(); }).detach();

    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> ur5e = std::make_shared<moveit_interface_cpp::MoveRobotClass>("ur_manipulator", node_movegroup);

    std::shared_ptr<HeatLogicNode> node2 = std::make_shared<HeatLogicNode>(ur5e,components,safety_configs);

    std::cout<<"Main thread is blocked "<<std::endl;
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
    
}


