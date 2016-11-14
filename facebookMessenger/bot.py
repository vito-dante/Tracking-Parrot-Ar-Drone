#! /usr/bin/python

import os
from flask import Flask, request
from pymessenger.bot import Bot
import rospy
import sys
sys.path.append('..')
from scripts.Drone.drone_commands import DroneCommands

rospy.init_node('Messenger')
drone_messenger = DroneCommands()

app = Flask(__name__)

VERIFY_TOKEN = os.getenv('VERIFY_TOKEN')
ACCESS_TOKEN = os.getenv('ACCESS_TOKEN')


bot = Bot(ACCESS_TOKEN)

@app.route("/", methods=['GET', 'POST'])
def send_message():
    if request.method == 'GET':
        if request.args.get("hub.verify_token") == VERIFY_TOKEN:
            return request.args.get("hub.challenge")
        else:
            return 'Invalid Url'

    if request.method == 'POST':
        output = request.get_json()
        for event in output['entry']:
            messaging = event['messaging']
            for x in messaging:
                if x.get('message'):
                    recipient_id = x['sender']['id']
                    if x['message'].get('text'):
                        message = x['message']['text']
                        if command_valid(message):
                            command_sender[message]()
                            bot.send_text_message(recipient_id, "mensaje enviado")
                        else:
                            bot.send_text_message(recipient_id, "mensaje invalido")
                    if x['message'].get('attachment'):
                        bot.send_attachment_url(recipient_id, x['message']['attachment']['type'],
                                                x['message']['attachment']['payload']['url'])
                else:
                    pass
        return "Success"

command_sender = {
    "bateria": drone_messenger.show_battery,
    "despegar": drone_messenger.send_takeoff,
    "luces": drone_messenger.led_animation,
    "aterrizar": drone_messenger.send_land,
    "cambiar camara": drone_messenger.change_camera
}

def command_valid(word):
    word = word.lower()
    if word in command_sender:
        return True
    return False


if __name__ == "__main__":
    app.run(port=5000, debug=True)
