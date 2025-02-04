import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import ollama
from colorama import Fore
import time
from custom_interfaces.msg import Llmfeedback

class cobot_llm(Node):


    def __init__(self):
        super().__init__('cobot_llm')

        # main prompt setting up 
        component = open("/home/i40lab/workspace/ai_Redgio_ws/src/llm_communicator/resource/ai_redgo.txt", "r")
        # intent_selecter = open("/home/shayan/ur5/src/llm_communicator/resource/intent_classifier.txt", "r")
        id_detection = open("/home/i40lab/workspace/ai_Redgio_ws/src/llm_communicator/resource/id_selection.txt", "r")
        self.component_prompt = component.read()
        self.id_prompt = id_detection.read()
        # self.intent_select = intent_selecter.read()
        
        self.publisher_robot_listen_ = self.create_publisher(String, '/robot_listening', 10)
        self.publisher_id_ = self.create_publisher(Llmfeedback,'/com_id', 10)
        self.publisher_notif_ = self.create_publisher(String, '/notif_llm', 10)

        self.subscription_notif = self.create_subscription(
            String,
            '/notif_req',
            self.recieved_notif,
            10  # QoS history depth
        )


        self.subscription_listen = self.create_subscription(
            String,
            '/system_need',
            self.record_order_callback,
            10  # QoS history depth
        )       
        
        self.subscription_command_recieved = self.create_subscription(
            String,
            '/received_command',
            self.command_callback,
            10  # QoS history depth
        )

    def number_detect(self, value):
        for word in value.split(' '):
            if word in ['1', '2', '3', '4', '5', '6', 'one', 'two', 'three', 'four', 'five', 'six']:
                return 1
        return 2

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
    
    def record_order_callback(self, msg):
        msg = String()
        msg.data = 'activate'
        self.publisher_robot_listen_.publish(msg)



    def command_callback(self, msg):
        print('message recieved: ' + msg.data)

        
        
        # stream = ollama.chat(
        # model='llama3.2',
        # messages=[
        #     {
        #     'role': 'system',
        #     'content': self.intent_select
        #     },
        #     {'role': 'user', 'content': msg.data}],
        # )
        ans = Llmfeedback()
        # if not stream['message']['content'].isnumeric():
        #     ans.number_id = -1
        #     ans.type = 'Failed'
        #     self.publisher_id_.publish(ans)

        intent = self.number_detect(msg.data)
        print('the detected intent: ' + str(intent))
        ans.number_id = -1
        if intent == 1:
            stream = ollama.chat(
            model='llama3.2',
            messages=[
                {
                'role': 'system',
                'content': self.id_prompt
                },
                {'role': 'user', 'content': msg.data}],
            )
            ans.type = 'ComponentCounter'
        elif intent == 2:  
            stream = ollama.chat(
            model='llama3.2',
            messages=[
                {
                'role': 'system',
                'content': self.component_prompt
                },
                {'role': 'user', 'content': msg.data}],
            )
            ans.type = 'ComponentClass'
        else:
            ans.type = 'fail'

        ans.number_id = int(stream['message']['content'])
        print(ans)
        self.publisher_id_.publish(msg)


    # def selecting_id_callback(self, msg):

    #     stream = ollama.chat(
    #     model='llama3.2',
    #     messages=[
    #         {
    #         'role': 'system',
    #         'content': self.id_prompt
    #         },
    #         {'role': 'user', 'content': msg.data}],
    #     )
    #     msg = String()
    #     msg.data = stream['message']['content']
    #     print(msg.data)


    #     self.publisher_component_id_.publish(msg)

########################################### Node main function ###########################################
######################################################################################

def main(args=None):
    rclpy.init(args=args)
    
    # Create a subscriber node
    bot = cobot_llm()

    # Spin to keep the node running and responsive to callbacks
    rclpy.spin(bot)

    # Shutdown after exiting
    bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
