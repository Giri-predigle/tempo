import json
import os
from scripts.DB_config import Db_config
import pandas as pd
from pymysql import *
import pandas.io.sql as sql

api_config_db = Db_config()


class CompanyCard:

    def __init__(self, company_id=None):
        self.company_id = company_id
        return

    def viewCompany(self):
        company_id = self.company_id
        con = connect(user=api_config_db['USERNAME'], password=api_config_db['PASSWORD'],
                      host=api_config_db['HOST'], database=api_config_db['DATABASE'])
        sql_company_card = """SELECT d.company_name as Cname, d.domain_name as website, d.employee_size, 
        d.revenue_range as revenue,  d.industry_name as industry, d.prospect_count as prospectcount , c.company_link 
        as linkedin, cou.country_name as country FROM tbl_domain_intent_score d join tbl_company c on d.company_id = 
        c.company_id join tbl_country cou on c.country = cou.country_id where d.id = %s""" % company_id
        companyinfo = sql.read_sql(sql_company_card, con)
        con.close()
        companyinfo = companyinfo.to_dict('records')
        return companyinfo
