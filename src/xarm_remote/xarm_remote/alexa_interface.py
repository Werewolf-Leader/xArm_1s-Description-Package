#!/usr/bin/env python3

from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter

app = Flask(__name__)

from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard


from ask_sdk_core.dispatch_components import AbstractExceptionHandler


import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from xarm_remote.action import XarmTask

import threading
import time
import os

# Global variables for ROS2
ros_node = None
action_client = None
executor = None

def init_ros():
    """Initialize ROS2 in a separate thread"""
    global ros_node, action_client, executor
    
    rclpy.init()
    ros_node = Node("alexa_interface")
    action_client = ActionClient(ros_node, XarmTask, "task_server_number")
    
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    # Wait for action server to be available
    ros_node.get_logger().info("Waiting for action server...")
    action_client.wait_for_server()
    ros_node.get_logger().info("Action server connected!")
    
    # Spin in this thread
    executor.spin()

# Start ROS2 in background thread
ros_thread = threading.Thread(target=init_ros, daemon=True)
ros_thread.start()

# Give ROS2 time to initialize
time.sleep(2)




def send_goal_safe(task_number):
    """Safely send a goal to the action server"""
    try:
        if action_client is None:
            print(f"Action client not initialized, cannot send goal {task_number}")
            return False
        
        goal = XarmTask.Goal()
        goal.task_number = task_number
        future = action_client.send_goal_async(goal)
        return True
    except Exception as e:
        print(f"Error sending goal {task_number}: {e}")
        return False

def send_goal_sequence(task_numbers, delay=0.5):
    """Send multiple goals in sequence with delay"""
    def _send():
        for task_num in task_numbers:
            send_goal_safe(task_num)
            time.sleep(delay)
    
    threading.Thread(target=_send, daemon=True).start()


class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, How can I Help!"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)

        send_goal_safe(0)
        return handler_input.response_builder.response 

class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay I am Moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)

        # Send goals in sequence: home -> open gripper -> pose 1 -> close gripper
        send_goal_sequence([0, 11, 1, 10], delay=3.0)

        return handler_input.response_builder.response

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay See You Later Ciao"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)

        send_goal_safe(0)

        return handler_input.response_builder.response


class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hola, How can i Help !"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)

        send_goal_safe(0)
        
        return handler_input.response_builder.response





class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Invalid Input , I can't understand You!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response





skill_builder = SkillBuilder()
# Register your intent handlers to the skill_builder object
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

# Load Alexa Skill ID from environment variable
ALEXA_SKILL_ID = os.getenv('ALEXA_SKILL_ID')
if not ALEXA_SKILL_ID:
    raise ValueError("ALEXA_SKILL_ID environment variable is not set. Please set it before running.")

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id=ALEXA_SKILL_ID, app=app)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()

skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run()