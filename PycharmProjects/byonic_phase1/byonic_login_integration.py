import hashlib
import json
from datetime import date
import time
import logging
import redis
from flask import Flask, jsonify, request, Response
from flask_cors import CORS, cross_origin
from scripts.passwordreset import passwordReset
from scripts.DB_config import get_port, get_login_config
from dateutil.relativedelta import relativedelta
from scripts import response
import os
from pathlib import Path
from scripts import userdata_mgmt as user_data
from scripts import login as logincheck

APP = Flask(__name__)
cors = CORS(APP, resources={r"/api/*": {"origins": "*"}})
APP.config['CORS_HEADERS'] = 'Content-Type'
r = redis.StrictRedis(host='localhost', port=6379, db=0)


@APP.route('/api/login', methods=['POST'])
@cross_origin()
def login_user():
    if request.method == 'POST':

        content = request.get_json()
        username = content.get('username').lower()
        userpassword = content.get('password')
        # userpassword = hashlib.sha256(userpassword.encode("utf-8")).hexdigest()

        usr_login = logincheck.Logindatabase(username, userpassword)
        user_details = usr_login.login_check

        if user_details:
            return jsonify(user_details)
        else:
            return jsonify("Username or password is not valid")
    return "Not a POST Function"


@APP.route('/api/resetpassword/<identifier>', methods=['PUT'])
@cross_origin()
def Reset_Password(identifier):
    if request.method == 'PUT':
        id = identifier
        content = request.get_json()
        usernewpassword = content.get('newpassword')
        reset = passwordReset(id, usernewpassword).UPDATE_NEW_PASSWORD()
        if reset:
            return jsonify('Password Reset Successful')


if __name__ == "__main__":
    PORT = get_port()

    APP.config['SECRET_KEY'], APP.config['EXPIRY_TIME'] = get_login_config()
    APP.run(host='0.0.0.0', port=PORT, threaded=True)
