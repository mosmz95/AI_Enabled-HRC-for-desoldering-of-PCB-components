import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk
from PIL import Image, ImageTk

# Component dictionary
COMPONENTS = {
    0: 'Capacitor', 1: 'IC', 2: 'LED', 3: 'Resistor', 4: 'battery', 5: 'buzzer',
    6: 'clock', 7: 'connector', 8: 'diode', 9: 'display', 10: 'fuse', 11: 'inductor',
    12: 'potentiometer', 13: 'relay', 14: 'switch', 15: 'transistor'
}

# ROS 2 Node
class ComponentPublisherNode(Node):
    def __init__(self):
        super().__init__('component_publisher')
        self.publisher = self.create_publisher(Int32, 'component_id', 10)

    def publish_component_id(self, component_id):
        msg = Int32()
        msg.data = component_id
        self.publisher.publish(msg)
        self.get_logger().info(f"Published component ID: {component_id}")

# GUI Application
class ComponentGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node

        # Initialize tkinter
        self.root = tk.Tk()
        self.root.title("Component Selector")

        # Set up GUI layout
        self.create_gui()

        self.root.mainloop()

    def create_gui(self):
        # Create a frame to hold the components
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True)

        # Load and display component images with buttons
        for idx, component_name in COMPONENTS.items():
            # Load image
            image_path = f'path/to/images/{component_name.lower()}.png'  # Ensure the images are named properly
            try:
                image = Image.open(image_path)
                image = image.resize((100, 100), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(image)
            except FileNotFoundError:
                # Placeholder if image is missing
                photo = tk.PhotoImage(width=100, height=100)

            # Create button
            button = tk.Button(
                frame, image=photo, text=component_name, compound=tk.TOP,
                command=lambda idx=idx: self.on_component_click(idx)
            )
            button.image = photo  # Keep a reference to avoid garbage collection
            button.grid(row=idx // 4, column=idx % 4, padx=10, pady=10)

    def on_component_click(self, component_id):
        # Callback when a component is clicked
        self.ros_node.publish_component_id(component_id)
        print(f"Button clicked for component ID: {component_id}")

# Main function
def main():
    rclpy.init()
    ros_node = ComponentPublisherNode()

    try:
        # Start the GUI
        ComponentGUI(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
