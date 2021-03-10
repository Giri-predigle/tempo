"""
    created-on : 8/17/20
"""
# This script is used by the filters on the left side for getting data from the ilead db for the filtering purpose.
# When you the input for auto fills, it uses this script to connect to the db
import os
import json
from scripts.DB_config import Db_config
from pymysql import *
import pandas.io.sql as sql

api_config_db = Db_config()


class Database:

    def __init__(self):
        """
        initialize variables
        """
        self.cwd = os.getcwd()
        config_file = self.cwd + '/config/config_file.json'

        with open(config_file, 'r') as file:
            api_config = json.load(file)
        file.close()

        if 'DebugMode' in api_config and api_config['DebugMode'].upper() == "TRUE":
            self.debug_mode = True
        else:
            self.debug_mode = False

    @property
    def database(self):
        """
        database connection
        :return:
        """
        #   database connection ilead authentication
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        """ 
        sql queries used for selecting in the db
        """
        sql_query_topic = "SELECT topic_name as topics from tbl_topic"
        sql_query_job = "select job_level_name as job_levels from tbl_job_level"
        sql_query_employee = "select employee_range_values as employee_sizes from tbl_employee_size_range"
        sql_query_country = "select country_name as countries from tbl_country"
        sql_query_industry = "select industry_name as industries from tbl_industry"
        # sql_query_job_title = "select job_title_name as job_titles from tbl_job_title"
        sql_query_revenue = "select revenue_range_values as revenues from tbl_revenue_range"
        """
        checking queries in ilead database, renames their column names to the right
        """
        topic = sql.read_sql(sql_query_topic, con)
        # topic.rename(columns={"topic_name": "topics"}, inplace=True)
        job = sql.read_sql(sql_query_job, con)
        # job.rename(columns={"job_level_name": "job_levels"}, inplace=True)
        employee_size = sql.read_sql(sql_query_employee, con)
        # employee_size.rename(columns={"employee_range_values": "employee_sizes"}, inplace=True)
        country = sql.read_sql(sql_query_country, con)
        # country.rename(columns={"country_name": "countries"}, inplace=True)
        industry = sql.read_sql(sql_query_industry, con)
        # industry.rename(columns={'industry_name': 'industries'}, inplace=True)
        # jobtitle = sql.read_sql(sql_query_job_title, con)
        # jobtitle.rename(columns={"job_title_name": "job_titles"}, inplace=True)
        revenue = sql.read_sql(sql_query_revenue, con)
        # revenue.rename(columns={"revenue_range_values": "revenues"}, inplace=True)
        """
        storing database results in python dictionary
        """
        topic_list = topic.to_dict('list')
        job_list = job.to_dict('list')
        employee_size_list = employee_size.to_dict('list')
        country_list = country.to_dict('list')
        industry_list = industry.to_dict('list')
        # job_title_list = jobtitle.to_dict('list')
        revenue_list = revenue.to_dict('list')
        resp = {key: topic_list.get(key, [])
                     + job_list.get(key, [])
                     + employee_size_list.get(key, [])
                     + country_list.get(key, [])
                     + industry_list.get(key, [])
                     + revenue_list.get(key, [])
                for key in set(list(topic_list.keys()) + list(job_list.keys())
                               + list(employee_size_list.keys()) + list(country_list.keys())
                               + list(industry_list.keys()) + list(revenue_list.keys()))}
        # print(resp)
        return resp
