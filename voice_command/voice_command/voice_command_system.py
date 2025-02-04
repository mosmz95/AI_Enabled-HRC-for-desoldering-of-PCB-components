#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import queue
import sounddevice as sd
import vosk
import sys
from colorama import init, Fore, Style

q = queue.Queue()


def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))



class VoiceCommandSystem(Node):
    def __init__(self):
        # Initialize the node with the name 'voice_command_system'
        super().__init__('voice_command_system')

        # Create a publisher on the topic 'received_command'
        self.publisher_ = self.create_publisher(String, 'received_command', 10)
        self.initialization()
        # Create a subscription to the /received_command topic
        self.subscription = self.create_subscription(
            String,
            '/robot_listening',
            self.listener_callback,
            10  # QoS history depth
        )
        self.subscription  # prevent unused variable warning

        # Call the voice_command_system() function at a regular interval (1 Hz)
        # self.timer = self.create_timer(1.0, self.voice_command_system)
        # Predefined commands
        self.commands = ["move up", "move down", "turn on", "turn off"]
    
    def initialization(self):
        """
        This function will be called periodically (every second).
        It simulates receiving a voice command and publishes it to the 'received_command' topic.
        """
        model_path = "/home/i40lab/Downloads/vosk-model-en-us-0.42-gigaspeech/"

        try:
            if model_path is None:
                model_path = "model"
            if not os.path.exists(model_path):
                print ("The model does not exists")
            device_info = sd.query_devices(0, 'input')
            # soundfile expects an int, sounddevice provides a float:
            self.samplerate = int(device_info['default_samplerate'])

            self.model = vosk.Model(model_path)
            self.get_logger().info(Fore.YELLOW + f"Watting for record order\n")
                    
        except KeyboardInterrupt:
            print('\nDone')        


    def publishing_msg(self, command):
        self.get_logger().info(Fore.BLUE + f"Command detected: {command}\n")
        # Create a message to publish
        msg = String()
        msg.data = command

        # Publish the message
        self.publisher_.publish(msg)
        # Log the published message

        self.get_logger().info(Fore.GREEN + f'Publishing: "{msg.data}"\n')
        # self.get_logger().info(Fore.YELLOW + "Waiting for command\n")
    
    def listener_callback(self, msg):
        self.voice_command_system()




    def voice_command_system(self):
        try:

            with sd.RawInputStream(samplerate=self.samplerate, blocksize = 36000, device=5, dtype='int16',
                                    channels=1, callback=callback):
                    print('#' * 80)
                    print('Press Ctrl+C to stop the recording')
                    print('#' * 80)

                    rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                    self.get_logger().info(Fore.YELLOW + "Waiting for command\n")

                    while True:
                        data = q.get()
                        if rec.AcceptWaveform(data):
                            found_command = False
                            input_voice = rec.Result()[14:-3]
                            if not (input_voice == "" or input_voice=="the"):
                                print(input_voice)
                                self.publishing_msg(command=input_voice)
                                self.get_logger().info(Fore.YELLOW + f"Watting for record order\n")

                                break

                            # for command in self.commands:
                                # if command == input_voice:
                            # self.publishing_msg(command=command)
                                    # found_command = True
                            # if not found_command:
                                # self.get_logger().info(Fore.YELLOW + "Recieved wrong command ....\n" )

                                # self.get_logger().info(Fore.YELLOW + "Waiting for command\n")

                    
        except KeyboardInterrupt:
            print('\nDone')        


def main(args=None):
    # Initialize the ROS 2 Python Client Library
    rclpy.init(args=args)
    
    # Create the node instance
    node = VoiceCommandSystem()

    # Spin the node so it keeps executing and processing callbacks
    rclpy.spin(node)

    # Clean up when the node is stopped
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

