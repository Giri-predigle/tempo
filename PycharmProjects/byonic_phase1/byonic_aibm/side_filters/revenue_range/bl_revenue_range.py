from byonic_aibm.side_filters.revenue_range.co_revenue_range import CORevenue, COMLRevenue
from byonic_aibm.side_filters.revenue_range.da_revenue_range import DARevenue
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLRevenue(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLRevenue.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_revenue_range: DARevenue = DARevenue()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLRevenue.__logger

    def get_revenue_ranges(self):
        revenue_ranges = self.da_revenue_range.load_all()
        if revenue_ranges is not None:
            for revenue_range_key in revenue_ranges.keys():
                revenue_range = revenue_ranges[revenue_range_key]
                self.cache.put(revenue_range_key, revenue_range)
        return revenue_ranges

    def get_revenue_range(self, identifier):
        revenue_range_key = CORevenue.create_key(identifier=identifier)
        revenue_range = self.cache.get(revenue_range_key)
        if revenue_range is None:
            revenue_range = self.da_revenue_range.load_revenue_range(identifier)
            self.validate_revenue_range(revenue_range)
            self.cache.put(revenue_range.get_key(), revenue_range)
        return revenue_range

    def create_revenue_range(self, revenue_range):
        self.validate_revenue_range(revenue_range)
        self.da_revenue_range.add_revenue_range(revenue_range)
        if revenue_range.identifier is None:
            message = COMLRevenue.MSG_REVENUE_RANGE_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return revenue_range

    def update_revenue_range(self, revenue_range):
        self.validate_revenue_range(revenue_range)
        self.da_revenue_range.update_revenue_range(revenue_range)
        self.cache.remove(revenue_range.get_key())
        return revenue_range

    def delete_revenue_range(self, identifier):
        self.da_revenue_range.delete_revenue_range(identifier)
        revenue_range_key = CORevenue.create_key(identifier=identifier)
        self.cache.remove(revenue_range_key)
        return identifier

    def save_revenue_ranges(self):
        for revenue_range in self.get_revenue_ranges():
            self.save_revenue_range(revenue_range)

    def save_revenue_range(self, revenue_range):
        self.validate_revenue_range(revenue_range)
        if revenue_range.mark_for_deletion:
            self.delete_revenue_range(revenue_range.identifier)
        elif revenue_range.touched:
            if revenue_range.identifier is None:
                self.create_revenue_range(revenue_range)
            else:
                self.update_revenue_range(revenue_range)
            revenue_range.touched = False

    def validate_revenue_range(self, revenue_range):
        if not isinstance(revenue_range, CORevenue):
            message = COMLRevenue.MSG_REVENUE_RANGE_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
