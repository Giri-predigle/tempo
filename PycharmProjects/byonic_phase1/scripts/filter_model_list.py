import json
import os
from pymysql import *
import pandas.io.sql as sql
from scripts.DB_config import Db_config
# from scripts.byonic_data_access.db_connection_config import DBConnection

api_config_db = Db_config()
global topics, industries, name


class MLModel:

    def __init__(self, req_industry=None, req_topic=None, req_employee=None,
                 req_job=None, req_country=None, req_jobtitle=None, req_revenue=None,
                 req_domain=None, req_intentgrade=None
                 ):
        """

        :param req_industry:
        :param req_topic:
        :param req_employee:
        :param req_job:
        :param req_country:
        :param req_jobtitle:
        :param req_revenue:
        :param req_intentgrade:
        """
        # print(req_intentgrade)
        # self.cwd = current_working_dir
        if req_industry is None:
            req_industry = ["IT-Consulting", "Managed-Services-Provider"]

        self.req_topic = req_topic
        self.req_industry = req_industry
        self.req_country = req_country
        self.req_job = req_job
        self.req_employee = req_employee
        self.req_jobtitle = req_jobtitle
        self.req_revenue = req_revenue
        self.req_intentgrade = req_intentgrade
        self.req_domain = req_domain

        self.cwd = os.getcwd()
        config_file = self.cwd + '/config/config_file.json'
        with open(config_file, 'r') as file:
            api_config = json.load(file)
        file.close()

        if 'industries' in api_config:
            self.default_industry = api_config['industries']
        if 'topics' in api_config:
            self.default_topic = api_config['topics']
        if "employee_sizes" in api_config:
            self.default_employee = api_config['employee_sizes']
        if "job_levels" in api_config:
            self.default_job = api_config['job_levels']
        if "countries" in api_config:
            self.default_country = api_config['countries']
        if "job_titles" in api_config:
            self.default_job_title = api_config['job_titles']
        if "revenues" in api_config:
            self.default_revenue = api_config['revenues']
        # if "intentgrades" in api_config:
        #     self.default_intentgrade = api_config['intentgrades']
        if 'DebugMode' in api_config and api_config['DebugMode'].upper() == "TRUE":
            self.debug_mode = True
        else:
            self.debug_mode = False

    def model(self):

        """
        :return:
        """

        global topics, name, industries
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        print(api_config_db['DATABASE'])
        #
        sql_query_table = "SELECT id, topic_name as topic, domain_name as domain, employee_size," \
                          "iLead_score, surge_score, composite_score, country_name as country," \
                          "industry_name as industry, revenue_range as revenue, " \
                          "lead_count as techno_score FROM tbl_domain_intent_score "

        req_domain = self.req_domain
        # print(req_domain)
        req_industry = self.req_industry
        req_topic = self.req_topic
        req_employee = self.req_employee
        # req_job = self.req_job
        req_country = self.req_country
        # req_jobtitle = self.req_jobtitle
        req_revenue = self.req_revenue
        req_intentgrade = self.req_intentgrade

        if len(req_topic) == 0:
            topic = ""
        else:
            s = ''
            for i in req_topic:
                s += "'" + i + "',"
                name = s.rstrip(s[-1])

            topic = """topic_name in (""" + "".join(str(name)) + """)"""

        if len(req_domain) == 0:
            domains = ""
        else:
            s = ''
            for i in req_domain:
                s += "'" + i + "',"
                name = s.rstrip(s[-1])

            domains = """domain_name not in (""" + "".join(str(name)) + """)"""

        if len(req_country) == 0:
            countries = ""
        else:
            s = ''
            for i in req_country:
                s += "'" + i + "',"
                name = s.rstrip(s[-1])

            countries = """country_name in (""" + "".join(str(name)) + """)"""

        if len(req_revenue) == 0:
            revenues = ""
        else:
            s = ''
            for i in req_revenue:
                s += "'" + i + "',"
                name = s.rstrip(s[-1])

            revenues = """revenue_range in (""" + "".join(str(name)) + """)"""

        if len(req_employee) == 0:
            emp_sizes = ""
        else:
            s = ''
            for i in req_employee:
                s += "'" + i + "',"
                name = s.rstrip(s[-1])

            emp_sizes = """employee_size in (""" + "".join(str(name)) + """)"""

        if len(req_intentgrade) == 0:
            intentgrades = ""
        else:
            intentgrades = """composite_score between """ + str(req_intentgrade[0]) + """ and """ + str(req_intentgrade[1]) + """ """
            print(intentgrades)
        if len(req_industry) == 0:
            industries = ""
        else:
            s = ''
            for i in req_industry:
                s += "'" + i + "',"
                name = s.rstrip(s[-1])

            industries = """industry_name in (""" + "".join(str(name)) + """)"""

        # sql_query_table = """
        #                   SELECT id, topic_name  AS topic, domain_name AS domain, employee_size,iLead_score,
        #                   surge_score,composite_score, country_name  AS country,industry_name AS industry,
        #                   revenue_range AS revenue,lead_count AS techno_score FROM tbl_iLead_score_predict_11_Feb
        #                   where """ + topic + """ and """ + domains + """ and """ + countries + """ and
        #                   """ + industries + """ and """ + emp_sizes + """ and """ + revenues + """ and
        #                   """ + intentgrades + """
        #                   ORDER BY surge_score DESC, count_0to3_months DESC,
        #                   prospect_count DESC
        #                   """
        print(sql_query_table)
        data = sql.read_sql(sql_query_table, con)
        # print(data.isnull().sum())
        print(data)
        con.close()

        data['sno'] = data.index
        col_name = "sno"
        first_col = data.pop(col_name)
        data.insert(0, col_name, first_col)
        data.columns = data.columns.str.lower()
        # print(data.columns)

        data['surge_score'].fillna(0, inplace=True)
        data['industry'].fillna("", inplace=True)
        data['topic'].fillna("", inplace=True)
        data['employee_size'].fillna("", inplace=True)
        data['country'].fillna("", inplace=True)
        data['revenue'].fillna("", inplace=True)
        data['composite_score'].fillna(1, inplace=True)

        industry_list = data['industry'].str.contains(r'\b(?:{})\b'.format('|'.join(req_industry)), na=False)
        data = data[industry_list]

        topic_list = data['topic'].str.contains(r'\b(?:{})\b'.format('|'.join(req_topic)), na=False)
        data = data[topic_list]

        employee_list = data['employee_size'].str.contains(r'\b(?:{})\b'.format('|'.join(req_employee)), na=False)
        data = data[employee_list]

        # job_level_list = data['job_level'].str.contains(r'\b(?:{})\b'.format('|'.join(req_job)))
        # data = data[job_level_list]

        country_list = data['country'].str.contains(r'\b(?:{})\b'.format('|'.join(req_country)), na=False)
        data = data[country_list]

        # job_title_list = data['job_title'].str.contains(r'\b(?:{})\b'.format('|'.join(req_jobtitle)))
        # data = data[job_title_list]

        data['revenue'] = data['revenue'].replace({r'\$': ''}, regex=True)

        d = {ord(x): "" for x in ":$"}
        req_revenue = [x.translate(d) for x in req_revenue]

        revenue_list = data['revenue'].str.contains(r'\b(?:{})\b'.format('|'.join(req_revenue)), na=False)
        data = data[revenue_list]

        # intentgrade_list = data['intentgrade'].str.contains(r'\b(?:{})\b'.format('|'.join(req_revenue)))
        # data = data[intentgrade_list]

        if req_intentgrade:
            data['composite_score'] = data['composite_score'].astype('float')

            compositescore_list = data['composite_score'].between(req_intentgrade[0], req_intentgrade[1],
                                                                  inclusive=True)
            data = data[compositescore_list]

        if req_domain:
            domain_list = data['domain'].str.contains(r'\b(?:{})\b'.format('|'.join(req_domain)), na=False)
            data = data[~domain_list]
        data = data.head(api_config_db['LIMIT'])
        complete_filter_data = data.to_dict('records')
        resp_data = {'Domains': complete_filter_data}
        return resp_data


if __name__ == '__main__':
    MLModel()
