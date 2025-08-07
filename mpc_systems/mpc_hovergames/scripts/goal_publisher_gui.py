import dearpygui.dearpygui as dpg

import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped


class GoalPublisherGUI:
    def __init__(self):
        # Initialize ROS publisher and goal message
        self.publisher = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = "map"
        self.goal_msg.pose.position.x = 0
        self.goal_msg.pose.position.y = 0
        self.goal_msg.pose.position.z = 1.6
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = 0
        self.goal_msg.pose.orientation.w = 1

        dpg.create_context()

        with dpg.window(label="Goal Publisher", width=1000, height=150):
            dpg.add_text("Enter goal position and orientation (ZYX Euler angles):")
            with dpg.group(horizontal=True):
                dpg.add_text("x: ")
                self.x_input = dpg.add_input_double(default_value=-3, width=100)
                dpg.add_text("y: ")
                self.y_input = dpg.add_input_double(default_value=-1, width=100)
                dpg.add_text("z: ")
                self.z_input = dpg.add_input_double(
                    default_value=1.6,
                    min_value=1,
                    max_value=2,
                    min_clamped=True,
                    max_clamped=True,
                    width=100,
                )
                dpg.add_text("phi: ")
                self.phi_input = dpg.add_input_double(default_value=0, width=100)
                dpg.add_text("theta: ")
                self.theta_input = dpg.add_input_double(default_value=0, width=100)
                dpg.add_text("psi: ")
                self.psi_input = dpg.add_input_double(default_value=0, width=100)
            dpg.add_button(label="Publish Goal", callback=self.publish_goal)
            dpg.add_separator()
            dpg.add_button(label="Close", callback=self.close)

        dpg.create_viewport(title="Goal Publisher", width=1100, height=150)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.start_dearpygui()
        dpg.destroy_context()

    def publish_goal(self, sender, app_data, user_data):
        # Obtain current input field values
        x = dpg.get_value(self.x_input)
        y = dpg.get_value(self.y_input)
        z = dpg.get_value(self.z_input)
        phi = dpg.get_value(self.phi_input)
        theta = dpg.get_value(self.theta_input)
        psi = dpg.get_value(self.psi_input)

        # Convert ZYX Euler angles to quaternion for ROS
        q = tft.quaternion_from_euler(psi, theta, phi, axes="rzyx")

        # Create goal message
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.pose.position.x = x
        self.goal_msg.pose.position.y = y
        self.goal_msg.pose.position.z = z
        self.goal_msg.pose.orientation.x = q[0]
        self.goal_msg.pose.orientation.y = q[1]
        self.goal_msg.pose.orientation.z = q[2]
        self.goal_msg.pose.orientation.w = q[3]

        # Publish goal
        self.publisher.publish(self.goal_msg)

        # Print goal confirmation
        print(
            f"Goal published: {{x: {x}, y: {y}, z: {z}, phi: {phi}, theta: {theta}, psi: {psi}}}"
        )

    def close(self, sender, data):
        dpg.stop_dearpygui()


if __name__ == "__main__":
    rospy.init_node("goal_publisher", anonymous=True)
    gui = GoalPublisherGUI()
