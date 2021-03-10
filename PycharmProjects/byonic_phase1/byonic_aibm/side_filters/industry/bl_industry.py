from byonic_aibm.side_filters.industry.co_industry import COIndustry
from byonic_aibm.side_filters.industry.co_industry import COMLIndustry

from byonic_aibm.side_filters.industry.da_industry import DAIndustry
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLIndustry(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLIndustry.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_industry: DAIndustry = DAIndustry()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLIndustry.__logger

    def get_industries(self):
        industries = self.da_industry.load_all()
        if industries is not None:
            for industry_key in industries.keys():
                industry = industries[industry_key]
                self.cache.put(industry_key, industry)
        return industries

    def get_industry(self, identifier):
        industry_key = COIndustry.create_key(identifier=identifier)
        industry = self.cache.get(industry_key)
        if industry is None:
            industry = self.da_industry.load_industry(identifier)
            self.validate_industry(industry)
            self.cache.put(industry.get_key(), industry)
        return industry

    def create_industry(self, industry):
        self.validate_industry(industry)
        self.da_industry.add_industry(industry)
        if industry.identifier is None:
            message = COMLIndustry.MSG_INDUSTRY_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return industry

    def update_industry(self, industry):
        self.validate_industry(industry)
        self.da_industry.update_industry(industry)
        self.cache.remove(industry.get_key())
        return industry

    def delete_industry(self, identifier):
        self.da_industry.delete_industry(identifier)
        industry_key = COIndustry.create_key(identifier=identifier)
        self.cache.remove(industry_key)
        return identifier

    def save_industries(self):
        for industry in self.get_industries():
            self.save_industry(industry)

    def save_industry(self, industry):
        self.validate_industry(industry)
        if industry.mark_for_deletion:
            self.delete_industry(industry.identifier)
        elif industry.touched:
            if industry.identifier is None:
                self.create_industry(industry)
            else:
                self.update_industry(industry)
            industry.touched = False

    def validate_industry(self, industry):
        if not isinstance(industry, COIndustry):
            message = COMLIndustry.MSG_INDUSTRY_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
