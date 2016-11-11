#! /usr/bin/python

import os
from flask import Flask, request
from pymessenger.bot import Bot

app = Flask(__name__)

VERIFY_TOKEN = os.getenv('VERIFY_TOKEN')
ACCESS_TOKEN = os.getenv('ACCESS_TOKEN')

bot = Bot(ACCESS_TOKEN)

@app.route("/", methods=['GET', 'POST'])
def hello():
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
                        if message == 'Hi':
                            bot.send_text_message(recipient_id, 'Hi how are you\
                                                  doinng')
                        else:
                            bot.send_text_message(recipient_id, message)
                        print(message)
                    if x['message'].get('attachment'):
                        bot.send_attachment_url(recipient_id, x['message']['attachment']['type'],
                                                x['message']['attachment']['payload']['url'])
                else:
                    pass
        return "Success"


if __name__ == "__main__":
    app.run(port=5000, debug=True)
