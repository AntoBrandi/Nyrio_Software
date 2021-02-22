#!/usr/bin/env python
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
import rospy
import threading
from tasks import Wake, Sleep


threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True)).start()


app = Flask(__name__)
ask = Ask(app, "/")


@ask.launch
def launch():
    # wake the robot and bring it in the home position
    task = Wake()
    task.execute()
    msg = render_template('online')
    return question(msg)


@ask.intent("SleepIntent")
def sleep():
    # sleep the robot and bring it in the release position
    task = Sleep()
    task.execute()
    msg = render_template('sleep')
    return statement(msg)


@ask.intent("AMAZON.FallbackIntent")
def fallback():
    # unexpected action required
    fallback_msg = render_template('fallback')
    return statement(fallback_msg)


if __name__ == '__main__':
    app.run()