#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String
from pynput import keyboard

class KeyboardRRTControlNode:
    def __init__(self, controller_side='right'):
        rospy.init_node('keyboard_rrt_control_node', anonymous=True)
        self.publisher_signal = rospy.Publisher(f'/{controller_side}/planning_signal', Int32, queue_size=10)
        self.publisher_target = rospy.Publisher(f'/{controller_side}/target_frame', String, queue_size=10)
        
        rospy.loginfo(f"Keyboard RRT Control Node started for {controller_side} side.")
        rospy.loginfo("Press '1' to start RRT, '0' to stop RRT.")
        rospy.loginfo("Press 'n' or 'p' to navigate frames, 'm' or 'k' to navigate tasks, and 'r' to confirm selection.")
        
        # Task and frame navigation
        self.object_frames = [["task1_orange_can", "task1_basket_target"],
                              ["task2_lower", "task2_upper"],
                              ["task3_blue", "task3_blue_target"],
                              ["task4_clothes", "task4_candy", "task4_can", "task4_clothes_target", "task4_candy_target", "task4_can_target"]]
        self.current_task_index = 0
        self.current_frame_index = 0
        self.selected_frame = self.object_frames[self.current_task_index][self.current_frame_index]

    def update_target_frame(self):
        """
        Update the selected frame based on the current indices.
        """
        self.selected_frame = self.object_frames[self.current_task_index][self.current_frame_index]
        rospy.loginfo(f"Current Target Frame: {self.selected_frame} (Task {self.current_task_index + 1}, Frame {self.current_frame_index + 1})")

    def on_press(self, key):
        """
        Callback function for key press events.
        """
        try:
            if key.char == '1':
                rospy.loginfo("Key '1' pressed. Starting RRT.")
                self.publisher_signal.publish(1)
            elif key.char == '0':
                rospy.loginfo("Key '0' pressed. Stopping RRT.")
                self.publisher_signal.publish(0)
            elif key.char == 'n':  # Next frame
                self.current_frame_index = (self.current_frame_index + 1) % len(self.object_frames[self.current_task_index])
                self.update_target_frame()
            elif key.char == 'p':  # Previous frame
                self.current_frame_index = (self.current_frame_index - 1) % len(self.object_frames[self.current_task_index])
                self.update_target_frame()
            elif key.char == 'm':  # Next task
                self.current_task_index = (self.current_task_index + 1) % len(self.object_frames)
                self.current_frame_index = 0  # Reset frame index for the new task
                self.update_target_frame()
            elif key.char == 'k':  # Previous task
                self.current_task_index = (self.current_task_index - 1) % len(self.object_frames)
                self.current_frame_index = 0  # Reset frame index for the new task
                self.update_target_frame()
            elif key.char == 'r':  # Enter key to confirm selection
                rospy.loginfo(f"Target Frame '{self.selected_frame}' confirmed.")
                self.publisher_target.publish(self.selected_frame)
        except AttributeError:
            # Handle special keys if needed
            pass

    def on_release(self, key):
        """
        Callback function for key release events.
        """
        if key == keyboard.Key.esc:
            # Stop listener on 'ESC'
            rospy.loginfo("ESC pressed. Exiting keyboard listener.")
            return False

    def listen_keyboard(self):
        """
        Listen to keyboard events using pynput's Listener.
        """
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()  # Block until listener stops

if __name__ == "__main__":
    try:
        # Pass the controller side as needed
        node = KeyboardRRTControlNode(controller_side='right')
        node.listen_keyboard()
    except rospy.ROSInterruptException:
        pass
