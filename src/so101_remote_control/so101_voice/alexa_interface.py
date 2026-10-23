#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from so101_msgs.action import So101Task
import rclpy
from rclpy.node import Node
import threading
from rclpy.action import ActionClient

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node('alexa_interface'), So101Task, "task_server")

app = Flask(__name__)


class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Initializing so101 manipulator"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(
            False)

        goal = So101Task.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "picking up the object"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)

        goal = So101Task.Goal()
        goal.task_number = 1
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response



class HomeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("HomeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Moving to rest position, see you later"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Home", speech_text)).set_should_end_session(
            True)

        goal = So101Task.Goal()
        goal.task_number = 2
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, I am ready"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)
            
        goal = So101Task.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response

        speech = "Hmm, I didn't catch that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(HomeIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())


skill_adapter = SkillAdapter(
    skill=skill_builder.create(), 
    skill_id="amzn1.ask.skill.0219f999-8cf2-47f2-a700-cd0a6c5b5f0c",
    app=app)


skill_adapter.register(app=app, route="/")


if __name__ == '__main__':
    app.run()