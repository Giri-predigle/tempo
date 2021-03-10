from sqlalchemy import Column, String, Integer
from sqlalchemy.ext.declarative import declarative_base

ORMbase = declarative_base()


class CODomain_filter(ORMbase):
    # __tablename__ = "tbl_iLead_score_predict"
    __tablename__ = "tbl_domain_intent_score"

    col_identifier = Column('id', Integer, primary_key=True)
    col_topic_name = Column('topic_name', String)
    col_domain_name = Column('domain_name', String)
    col_employee_size = Column('employee_size', String)
    col_iLead_score = Column('iLead_score', Integer, quote=False)
    col_surge_score = Column('surge_score', Integer)
    col_composite_score = Column('composite_score', Integer)
    col_country_name = Column('country_name', String)
    col_industry_name = Column('industry_name', String)
    col_revenue_range = Column('revenue_range', String)
    col_prospect_count = Column('prospect_count', Integer)
    # col_count_0to3_months = Column('count_0to3_months', Integer)

    def __init__(self, identifier, topic, empsize, industry, domain, revenue, compositescore, surgescore, prospectcount,
                 ileadscore, country):
        super().__init__()
        self.identifier = identifier
        self.topic = topic
        self.empsize = empsize
        self.industry = industry
        self.domain = domain
        self.revenue = revenue
        self.compositescore = compositescore
        self.surgescore = surgescore
        self.prospectcount = prospectcount
        self.country = country
        self.ileadscore = ileadscore

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return CODomain_filter.get_class_name() + str(self.identifier)

    @property
    def identifier(self):
        return self.col_identifier

    @identifier.setter
    def identifier(self, identifier):
        self.col_identifier = identifier

    @property
    def topic(self):
        return self.col_topic_name

    @topic.setter
    def topic(self, topic):
        self.col_topic_name = topic

    @property
    def empsize(self):
        return self.col_employee_size

    @empsize.setter
    def empsize(self, empsize):
        self.col_employee_size = empsize

    @property
    def industry(self):
        return self.col_industry_name

    @industry.setter
    def industry(self, industry):
        self.col_industry_name = industry

    @property
    def revenue(self):
        return self.col_revenue_range

    @revenue.setter
    def revenue(self, revenue):
        self.col_revenue_range = revenue

    @property
    def domain(self):
        return self.col_domain_name

    @domain.setter
    def domain(self, domain):
        self.col_domain_name = domain

    @property
    def compositescore(self):
        return self.col_composite_score

    @compositescore.setter
    def compositescore(self, compositescore):
        self.col_composite_score = compositescore

    @property
    def ileadscore(self):
        return self.col_iLead_score

    @ileadscore.setter
    def ileadscore(self, ileadscore):
        self.col_iLead_score = ileadscore

    @property
    def surgescore(self):
        return self.col_surge_score

    @surgescore.setter
    def surgescore(self, surgescore):
        self.col_surge_score = surgescore

    @property
    def prospectcount(self):
        return self.col_prospect_count

    @prospectcount.setter
    def prospectcount(self, prospectcount):
        self.col_prospect_count = prospectcount

    @property
    def country(self):
        return self.col_country_name

    @country.setter
    def country(self, country):
        self.col_country_name = country

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.topic,
                self.empsize,
                self.revenue,
                self.ileadscore,
                self.surgescore,
                self.prospectcount,
                self.compositescore,
                self.country,
                self.industry,
                self.domain
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'id': self.identifier,
                'topic': self.topic,
                'employee_size': self.empsize,
                'domain': self.domain,
                'industry': self.industry,
                'revenue': self.revenue,
                'ilead_score': self.ileadscore,
                'surge_score': self.surgescore,
                'composite_score': self.compositescore,
                'country': self.country,
                'prospectcount': self.prospectcount}


if __name__ == '__main__':
    print('hello')
