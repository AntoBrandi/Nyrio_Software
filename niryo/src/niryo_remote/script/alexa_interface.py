#!/usr/bin/env python
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
from niryo_remote.msg import NiryoTaskAction, NiryoTaskGoal
import rospy
import threading
import actionlib


threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True)).start()
client = actionlib.SimpleActionClient('task_server', NiryoTaskAction)

app = Flask(__name__)
ask = Ask(app, "/")


@ask.launch
def launch():
    # Function that gets called when the skill is activated
    goal = NiryoTaskGoal(task_number=0)
    client.send_goal(goal)
    msg = render_template('online')
    return question(msg)


@ask.intent("DanceIntent")
def dance():
    # Function that is called when the Dance Intent is activated
    goal = NiryoTaskGoal(task_number=1)
    client.send_goal(goal)
    msg = render_template('dance')
    return statement(msg)


@ask.intent("PickIntent")
def pick():
    # Function that is called when the Pick Intent is activated
    goal = NiryoTaskGoal(task_number=2)
    client.send_goal(goal)
    msg = render_template('pick')
    return statement(msg)


@ask.intent("SleepIntent")
def sleep():
    # Function that is called when the Sleep Intent is activated
    goal = NiryoTaskGoal(task_number=3)
    client.send_goal(goal)
    msg = render_template('sleep')
    return statement(msg)


@ask.intent("WakeIntent")
def sleep():
    # Function that is called when the Wake Intent is activated
    goal = NiryoTaskGoal(task_number=0)
    client.send_goal(goal)
    msg = render_template('wake')
    return statement(msg)


@ask.intent("AMAZON.FallbackIntent")
def fallback():
    fallback_msg = render_template('fallback')
    return statement(fallback_msg)


if __name__ == '__main__':
    client.wait_for_server()
    app.run()