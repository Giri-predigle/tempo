# from sqlalchemy import Column, String, Integer
# from scripts.byonic_data_access.db_connection_config import DBConnection
# from sqlalchemy.ext.declarative import declarative_base
#
# ORMbase = declarative_base()
#
#
# class Domain_filter(ORMbase):
#     __tablename__ = "tbl_domain_intent_score"
#
#     col_id = Column('id', Integer, primary_key=True)
#     col_topic_name = Column('topic_name', String)
#     col_domain_name = Column('domain_name', String)
#     col_employee_size = Column('employee_size', String)
#     col_iLead_score = Column('iLead_score', Integer, quote=False)
#     col_surge_score = Column('surge_score', Integer)
#     col_composite_score = Column('composite_score', Integer)
#     col_country_name = Column('country_name', String)
#     col_industry_name = Column('industry_name', String)
#     col_revenue_range = Column('revenue_range', String)
#     col_lead_count = Column('lead_count', Integer)
#
#
# if __name__ == '__main__':
#     data = DBConnection().getsession()
#     campaign_view = data.query(Domain_filter.col_id, Domain_filter.col_domain_name, Domain_filter.col_topic_name,
#                                Domain_filter.col_country_name, Domain_filter.col_industry_name,
#                                Domain_filter.col_iLead_score, Domain_filter.col_surge_score,
#                                Domain_filter.col_employee_size, Domain_filter.col_lead_count,
#                                Domain_filter.col_revenue_range, Domain_filter.col_composite_score).all()
#     # campaign_list = data.query(Domain_filter).filter(Domain_filter.col_id == 1).all()
#
#     values = ['id',  'domain_name', 'topic_name', 'country_name', 'industry_name', 'iLead_score', 'surge_score',
#               'employee_size', 'lead_count', 'revenue_range', 'composite_score']
#     dictionary = []
#
#     for i in range(len(campaign_view)):
#         files = dict(zip(values, campaign_view[i]))
#         dictionary.append(files)
#
#     resp_data = {'Domains': dictionary}
#     print(resp_data)
