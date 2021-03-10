import hashlib
import base64
import os
from scripts.DB_config import get_port
from scripts.byonic_data_access.db_connection_config import DBConnection
from sqlalchemy import Table, column
from sqlalchemy import select, update, delete
from scripts.DB_config import Db_config
from flask import jsonify
import jwt
import json
import hashlib
import pandas as pd
from pymysql import *
import pandas.io.sql as sql

api_config_db = Db_config()


class resetPassword:
    def __init__(self, id=None, usernewpassword=None, resetid=None):
        """
        :param id
        :param usernewpassword
        """
        # print(api_config_db)
        self.id = id
        if resetid:
            self.resetid = '1'
        else:
            self.resetid = '0'
        if usernewpassword:
            self.setpassword = hashlib.sha512(usernewpassword.encode("utf-8")).hexdigest()
            # self.userpassword = base64.b64decode(userpassword).decode("utf-8")
            # self.setpassword = usernewpassword
        return

    # @property
    def NEW_PASSWORD(self):
        id = self.id
        resetid = self.resetid
        # resetid = '0'
        newpassword = self.setpassword
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        sql_setpassword = "UPDATE tbl_byonic_user SET user_password = '%s', reset='%s' WHERE user_id = %s" % (newpassword, resetid, id)
        # sql_setpassword = "UPDATE tbl_byonic_user SET user_password = '%s'WHERE user_id = %s" % (
        # newpassword, id)
        cursor = con.cursor()
        cursor.execute(sql_setpassword)
        con.commit()
        con.close()
        return "Password updated success"


if __name__ == '__main__':
    resetPassword().NEW_PASSWORD()
    pass