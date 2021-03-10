"""
    created-on : 12/15/20
"""
# get user info from the DB
import hashlib
import json
import os
from scripts.DB_config import Db_config
from pymysql import *
import pandas.io.sql as sql

api_config_db = Db_config()


# from sqlalchemy import create_engine
# from sqlalchemy.orm import sessionmaker, scoped_session
class Userdatabase:

    def __init__(self, firstname=None, lastname=None, email_addr=None, id=None, password=None, status=None,
                 role_id=None, org_id=None):
        """
        :param firstname
        :param lastname
        :param email_addr
        :param id
        :param password
        """
        self.firstname = firstname
        self.lastname = lastname
        self.email_addr = email_addr
        self.id = id
        self.role = role_id
        self.org = org_id
        if password:
            self.password = hashlib.sha512(password.encode("utf-8")).hexdigest()
            # print(len(self.password))

        # self.password = password
        if status:
            self.status = 'Active'
        else:
            self.status = 'InActive'

        return

    @property
    def db_loader_user(self):
        org_id = self.org

        #   database connection ilead authentication
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])

        if org_id is None:
            sql_user_db = "SELECT tb.user_id as id, tb.user_first_name as firstname, tb.user_last_name as lastname," \
                          "tb.user_email as email_addr, tb.user_password as password, tbo.user_organisation as org_id, " \
                          "tr.user_role as role_id FROM tbl_byonic_user tb LEFT JOIN tbl_byonic_roles tr on " \
                          "tb.user_role_id = tr.user_role_id LEFT JOIN tbl_byonic_organisation tbo on " \
                          "tb.user_organisation_id = tbo.user_organisation_id"
        else:
            sql_user_db = "SELECT tb.user_id as id, tb.user_first_name as firstname, tb.user_last_name as lastname," \
                          "tb.user_email as email_addr, tb.user_password as password, tbo.user_organisation as org_id, " \
                          "tr.user_role as role_id FROM tbl_byonic_user tb LEFT JOIN tbl_byonic_roles tr on " \
                          "tb.user_role_id = tr.user_role_id LEFT JOIN tbl_byonic_organisation tbo on " \
                          "tb.user_organisation_id = tbo.user_organisation_id WHERE " \
                          "tb.user_organisation_id = %s" % org_id

        user_db = sql.read_sql(sql_user_db, con)
        user_db = user_db.to_dict('records')
        con.close()

        return user_db

    @property
    def UPDATE_user_db(self):
        firstname = self.firstname
        lastname = self.lastname
        email_addr = self.email_addr
        status = self.status
        id = self.id
        password = self.password
        print(password)
        #   database connection ilead authentication
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database= api_config_db['DATABASE'])
        # sql queries used for updating user in the db

        sql_update_row = "UPDATE tbl_byonic_user SET user_first_name = '%s', user_last_name = '%s' ," \
                         " user_email = '%s', user_password = '%s', Status = '%s' WHERE user_id = %s" \
                         % (firstname, lastname, email_addr, password, status, id)
        # print(sql_update_row)
        cursor = con.cursor()
        cursor.execute(sql_update_row)

        # query = "select * from tbl_byonic_user where user_id = " + str(id)
        # user_db = sql.read_sql(query, con)
        # user_db.to_dict('records')
        cursor.close()
        con.commit()
        con.close()
        return

    @property
    def INSERT_user_db(self):
        firstname = self.firstname
        lastname = self.lastname
        email_addr = self.email_addr
        status = self.status
        # id = self.id
        password = self.password
        # print(password)
        role = self.role
        org = self.org
        #   database connection ilead authentication
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])

        # sql queries used for adding user in the db

        sql_insert_row = "INSERT INTO tbl_byonic_user (user_first_name, user_last_name, user_email, user_password, " \
                         "Status, user_role_id, user_organisation_id) VALUES ('%s', '%s', '%s', '%s','%s', %s, %s) " \
                         % (firstname, lastname, email_addr, password, status, role, org)
        # print(sql_insert_row)
        cursor = con.cursor()
        cursor.execute(sql_insert_row)
        # result = cursor.execute(sql_insert_row)

        con.commit()
        query = "select user_id from tbl_byonic_user"
        user_db = sql.read_sql(query, con)

        user_db = user_db["user_id"].iloc[-1]

        query = "select * from tbl_byonic_user where user_id = %s" % str(user_db)
        user_db = sql.read_sql(query, con)

        cursor.close()
        con.close()

        return user_db.to_dict('records')

    @property
    def DELETE_user_db(self):
        #   database connection ilead authentication
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        id = self.id
        # print(id)
        # sql queries used for deleting user in the db
        sql_delete_row = "DELETE FROM tbl_byonic_user WHERE user_id = %s" % id
        cursor = con.cursor()
        cursor.execute(sql_delete_row)
        con.commit()
        cursor.close()
        con.close()
        return


class Organisation_db:

    def __init__(self, org_name=None):
        """
        :param org_name
        """
        self.org_name = org_name

    @property
    def View_organisation(self):
        sql_view_org = "SELECT user_organisation_id as value ,user_organisation as viewValue " \
                       "from tbl_byonic_organisation"
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        org_db = sql.read_sql(sql_view_org, con)
        con.close()
        view_org_db = org_db.to_dict('records')

        return view_org_db

    @property
    def Create_organisation(self):
        org_name = self.org_name
        sql_query_create_org = "INSERT INTO tbl_byonic_organisation (user_organisation) VALUES ('%s')" % org_name

        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        cursor = con.cursor()

        cursor.execute(sql_query_create_org)
        con.commit()
        cursor.close()
        con.close()
        return org_name

    @property
    def Delete_organisation(self):
        org_name = self.org_name

        sql_delete_row = "DELETE FROM tbl_byonic_organisation WHERE user_organisation = '%s'" % org_name

        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        cursor = con.cursor()
        cursor.execute(sql_delete_row)
        con.commit()
        cursor.close()
        con.close()
        return org_name


class Role_db:

    def __init__(self, role_name=None):
        """
        :param role_name
        """

        self.Role_name = role_name

    @property
    def View_role(self):
        sql_view_role = "SELECT user_role_id as value ,user_role as viewValue " \
                        "from tbl_byonic_roles"
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        role_db = sql.read_sql(sql_view_role, con)
        con.close()
        view_role_db = role_db.to_dict('records')

        return view_role_db

if __name__ == '__main__':
    Role_db().View_role()