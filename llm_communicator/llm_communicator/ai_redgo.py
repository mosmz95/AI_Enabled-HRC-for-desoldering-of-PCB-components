import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import ollama
from colorama import Fore
import time

from ament_index_python.packages import get_package_share_directory
package_share_directory = get_package_share_directory('llm_communicator')
print(package_share_directory)
folder_path = os.path.join(package_share_directory, "configs")
print(folder_path)

class cobot_llm(Node):


    def __init__(self,folder_path):
        super().__init__('cobot_llm')
        self.get_logger().info("LLM node has been started.")
        # main prompt setting up 
        component = open(os.path.join(folder_path, "ai_redgo.txt"), "r")
        id_detection = open(os.path.join(folder_path, "id_selection.txt"), "r")

        self.component_prompt = component.read()
        self.id_prompt = id_detection.read()
        
        self.publisher_component_ = self.create_publisher(String, '/focus_component', 10)
        self.publisher_component_id_ = self.create_publisher(String,'/component_id', 10) # component number for heating 
        self.publisher_notif_ = self.create_publisher(String, '/notif_llm', 10)

        self.subscription_notif = self.create_subscription(
            String,
            '/notif_req',
            self.recieved_notif,
            10  # QoS history depth
        )


        self.subscription_component_selection = self.create_subscription(
            String,
            '/component_selection',
            self.component_selection_callback,
            10  # QoS history depth
        )

        
        self.subscription_com_id = self.create_subscription(
            String,
            '/id_order',
            self.selecting_id_callback,
            10  # QoS history depth
        )

    def recieved_notif(self, msg):
        stream = ollama.chat(
        model='llama3.2',
        messages=[ {
            'role': 'system',
            'content': 'You are an AI assistant. Do not mention you are LLM mode. your name is cobot one. Give me short answers unless I ask you for long one, notify the user base on the recieved notification.',
            },
            {'role': 'user', 
            'content': msg.data}
            ],
        )
        msg = String()
        msg.data = stream['message']['content']

        self.publisher_notif_.publish(msg)


    def component_selection_callback(self, msg):

        stream = ollama.chat(
        model='llama3.2',
        messages=[
            {
            'role': 'system',
            'content': self.component_prompt
            },
            {'role': 'user', 'content': msg.data}],
        )
        msg = String()
        msg.data = stream['message']['content']
        print(msg.data)
        self.publisher_component_.publish(msg)


    def selecting_id_callback(self, msg):

        stream = ollama.chat(
        model='llama3.2',
        messages=[
            {
            'role': 'system',
            'content': self.id_prompt
            },
            {'role': 'user', 'content': msg.data}],
        )
        msg = String()
        msg.data = stream['message']['content']
        print(msg.data)


        self.publisher_component_id_.publish(msg)

########################################### Node main function ###########################################
######################################################################################

def main(args=None):
    rclpy.init(args=args)
    
    # Create a subscriber node
    bot = cobot_llm(folder_path)

    # Spin to keep the node running and responsive to callbacks
    rclpy.spin(bot)

    # Shutdown after exiting
    bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
