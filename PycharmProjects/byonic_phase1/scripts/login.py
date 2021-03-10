"""
    created-on : 12/15/20
"""
# get user info from the DB
import os
from scripts.DB_config import Db_config
import jwt
import json
import base64
import hashlib
from pymysql import *
import pandas.io.sql as sql

api_config_db = Db_config()


class Logindatabase:

    def __init__(self, username=None, userpassword=None):
        """
        :param username
        :param userpassword
        """
        print(api_config_db)
        self.username = username
        if userpassword:
            self.userpassword = hashlib.sha512(userpassword.encode("utf-8")).hexdigest()
            # self.userpassword = base64.b64decode(userpassword).decode("utf-8")
            # self.userpassword = userpassword
        return

    @property
    def login_check(self):

        username = self.username
        userpassword = self.userpassword

        #   database connection ilead authentication
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])

        # sql queries used for selecting user in the db

        sql_login_check = "SELECT user_id as id, reset," \
                          " user_role_id as role_id, user_organisation_id as org_id" \
                          " FROM tbl_byonic_user WHERE user_email = '%s' and user_password = '%s'" % (
                              username, userpassword)

        user_db = sql.read_sql(sql_login_check, con)

        if user_db.empty:
            print('its empty')
            user_db = user_db.to_dict('records')
            # print(user_db)
        else:
            role_id = int(user_db['role_id'])
            if role_id == 3:
                print('User')
            elif role_id == 2:
                print('Admin')
            elif role_id == 1:
                print('Super Admin')
            user_db = user_db.to_dict('records')
            # user_db = user_db.to_json()
            print(user_db)
            user_db = jwt.encode({'token': user_db}, "byonicsecret", algorithm="HS256")
            print(user_db)
        return user_db
