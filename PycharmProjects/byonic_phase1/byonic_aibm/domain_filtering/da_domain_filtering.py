# from sqlalchemy import or_
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_aibm.domain_filtering.co_domain_filtering import CODomain_filter


class DADomain_Filter:

    def __init__(self, req_industry=None, req_topic=None, req_employee=None,
                 req_job=None, req_country=None, req_revenue=None,
                 req_domain=None, req_intentgrade=None, req_domainEx=None, req_domainIn=None
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
        self.req_topic = req_topic
        self.req_industry = req_industry
        self.req_country = req_country
        self.req_job = req_job
        self.req_employee = req_employee
        self.req_revenue = req_revenue
        self.req_intentgrade = req_intentgrade
        self.req_domain = req_domain
        self.req_domainEx = req_domainEx
        self.req_domainIn = req_domainIn
        # print(self.req_employee)

        # country = ['India', 'United kingdom']
        # topic = []
        # domain = []

    def load_all_domain(self):
        Dictionary = dict()
        dataAccess = DBConnection().getsession()
        db_query = dataAccess.query(CODomain_filter)
        if self.req_topic is not None:
            if len(self.req_topic) != 0:
                db_query = db_query.filter(CODomain_filter.col_topic_name.in_(self.req_topic))
        if self.req_country is not None:
            if len(self.req_country) != 0:
                db_query = db_query.filter(CODomain_filter.col_country_name.in_(self.req_country))
        if self.req_employee is not None:
            if len(self.req_employee) != 0:
                db_query = db_query.filter(CODomain_filter.col_employee_size.in_(self.req_employee))
        # if self.req_job is not None:
        #     if len(self.req_job) != 0:
        #         db_query = db_query.filter(CODomain_filter.col_employee_size.in_(self.req_job))
        if self.req_industry is not None:
            if len(self.req_industry) != 0:
                db_query = db_query.filter(CODomain_filter.col_industry_name.in_(self.req_industry))
        if self.req_revenue is not None:
            if len(self.req_revenue) != 0:
                db_query = db_query.filter(CODomain_filter.col_revenue_range.in_(self.req_revenue))
        if self.req_intentgrade is not None:
            if len(self.req_intentgrade) != 0:
                db_query = db_query.filter(CODomain_filter.col_composite_score >= self.req_intentgrade[0]).\
                    filter(CODomain_filter.col_composite_score <= self.req_intentgrade[1])
        # if self.req_domain is not None:
        #     if len(self.req_domain) != 0:
        #         db_query = db_query.filter(CODomain_filter.col_domain_name.notin_(self.req_domain))
        if self.req_domainEx is not None:
            if len(self.req_domainEx) != 0:
                db_query = db_query.filter(CODomain_filter.col_domain_name.notin_(self.req_domainEx))
        if self.req_domainIn is not None:
            if len(self.req_domainIn) != 0:
                db_query = db_query.filter(CODomain_filter.col_domain_name.in_(self.req_domainIn))
        db_query = db_query.order_by(CODomain_filter.col_surge_score.desc())
        # db_query = db_query.limit(10000).all()
        try:
            for ct in db_query:
                Dictionary[ct.get_key()] = ct
            domains = [Dictionary[domain_key].serialize for domain_key in Dictionary]
            print(domains)
            return {'Domains': domains}
        except:
            print('not working')


if __name__ == "__main__":
    DADomain_Filter().load_all_domain()
