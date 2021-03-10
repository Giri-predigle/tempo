import hashlib
import base64
import os
from scripts.DB_config import get_port
from scripts.byonic_data_access.db_connection_config import DBConnection
from sqlalchemy import Table, column
from sqlalchemy import select, update, delete
from flask import jsonify
import jwt
import json
import hashlib
import pandas as pd
from pymysql import *
import pandas.io.sql as sql


# api_config_db = Db_config()


class passwordReset:
    def __init__(self, id=None, usernewpassword=None):
        """
        :param id
        :param usernewpassword
        """
        # print(api_config_db)
        self.DBSession = DBConnection().getsession()
        self.metadata = DBConnection().metadata()
        self.id = 30
        if usernewpassword:
            self.setpassword = hashlib.sha512(usernewpassword.encode("utf-8")).hexdigest()
            # self.userpassword = base64.b64decode(userpassword).decode("utf-8")
            # self.userpassword = userpassword
        return

    # @property
    def UPDATE_NEW_PASSWORD(self):
        id = self.id
        conn = DBConnection().getEngine()
        metadata = DBConnection().metadata()
        byonic_user = Table('tbl_byonic_user', metadata, autoload=True, autoload_with=conn)
        query = select([byonic_user.columns.user_role_id, byonic_user.columns.user_first_name]).where(byonic_user.columns.user_role_id==1)
        ResultProxy = conn.execute(query).fetchall()
        # ResultSet = pd.read_sql(query, con=conn).to_json(orient='records')
        conn.close()
        # print(ResultProxy.fetchall())

        values = ['user_role_id', 'user_first_name']
        dictionary = []
        print(ResultProxy)
        for i in range(len(ResultProxy)):
            files = dict(zip(values, ResultProxy[i]))
            dictionary.append(files)

        campaign_view = {'campaign_view': dictionary}
        print(campaign_view)
        return


if __name__ == '__main__':
    passwordReset().UPDATE_NEW_PASSWORD()
    pass
