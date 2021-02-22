#!/usr/bin/env python
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
import rospy
import threading
from tasks import Sleep, Home, MoveUp


threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True)).start()


app = Flask(__name__)
ask = Ask(app, "/")


@ask.launch
def launch():
    msg = render_template('online')
    return question(msg)


@ask.intent("HomeIntent")
def home():
    # wake the robot and bring it in the home position
    task = Home()
    task.execute()
    msg = render_template('home')
    return statement(msg)


@ask.intent("SleepIntent")
def sleep():
    # sleep the robot and bring it in the release position
    task = Sleep()
    task.execute()
    msg = render_template('sleep')
    return statement(msg)


@ask.intent("MoveUpIntent")
def moveUp():
    # sleep the robot and bring it in the release position
    task = MoveUp()
    task.execute()
    msg = render_template('moveUp')
    return statement(msg)


@ask.intent("AMAZON.FallbackIntent")
def fallback():
    # unexpected action required
    fallback_msg = render_template('fallback')
    return statement(fallback_msg)


if __name__ == '__main__':
    app.run()