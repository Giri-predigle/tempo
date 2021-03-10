from byonic_aibm.side_filters.country.da_country import DACountry
from byonic_aibm.side_filters.employee_size.da_employee_size import DAEmployee
from byonic_aibm.side_filters.industry.da_industry import DAIndustry
from byonic_aibm.side_filters.intent_topics.da_intent_topics import DATopic
from byonic_aibm.side_filters.revenue_range.da_revenue_range import DARevenue
from byonic_aibm.side_filters.job_levels.da_job_level import DAJobLevel


class Sidefilters(object):

    def __init__(self):
        self.da_intent_topics: DATopic = DATopic()
        self.da_country: DACountry = DACountry()
        self.da_industry: DAIndustry = DAIndustry()
        self.da_job_level: DAJobLevel = DAJobLevel()
        self.da_revenue_range: DARevenue = DARevenue()
        self.da_employee_size: DARevenue

    def load_all(self):
        countries = self.da
        emp_sizes = DAEmployee.load_all()
        industries = DAIndustry.load_all()
        topics = DATopic.load_all_topics()
        revenues = DARevenue.load_all_revenue()
        jobs = DAJobLevel.load_all_job_level()

        topic_list = topics
        job_list = jobs
        employee_size_list = emp_sizes
        country_list = countries
        industry_list = industries
        revenue_list = revenues

        resp = {key: topic_list.get(key, [])
                     + job_list.get(key, [])
                     + employee_size_list.get(key, [])
                     + country_list.get(key, [])
                     + industry_list.get(key, [])
                     + revenue_list.get(key, [])
                for key in set(list(topic_list.keys()) + list(job_list.keys())
                               + list(employee_size_list.keys()) + list(country_list.keys())
                               + list(industry_list.keys()) + list(revenue_list.keys()))}
        return resp

if __name__ == '__main__':
    Sidefilters.load_all_side_filters()