from sqlalchemy import Column, String, Integer

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class CORevenue(COBase, ORMBase):
    __tablename__ = "tbl_revenue_range"

    col_id = Column('revenue_range_id', Integer, primary_key=True)
    __revenue_ranges = Column('revenue_range_values', String)

    def __init__(self, identifier, revenue_ranges):
        super().__init__()
        self.identifier = identifier
        self.revenue_ranges = revenue_ranges

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return CORevenue.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return CORevenue.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.col_id

    @identifier.setter
    def identifier(self, identifier):
        self.col_id = identifier

    @property
    def revenue_ranges(self):
        return self.__revenue_ranges

    @revenue_ranges.setter
    def revenue_ranges(self, revenue_ranges):
        self.__revenue_ranges = revenue_ranges

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.revenue_ranges,
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'revenue_ranges': self.revenue_ranges
                }

    @property
    def serialize_by_range(self):
        return {
                'revenue_ranges': self.revenue_ranges
                }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        revenue_ranges = json_data['revenue_ranges']
        revenue_ranges = CORevenue(identifier=identifier, revenue_ranges=revenue_ranges)
        return revenue_ranges

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'revenue_ranges' not in json_data:
            return False
        return True


class COMLRevenue(COMessageLiteral):
    # Validation Error Messages
    MSG_REVENUE_RANGES_NOT_VALID = 'Revenue Ranges are not valid!'
    MSG_REVENUE_RANGES_NOT_FOUND = 'RevenueS Ranges are not found!'
    MSG_REVENUE_RANGE_NOT_VALID = 'Revenue Range is not valid!'
    MSG_REVENUE_RANGE_NOT_FOUND = 'Revenue Range is not found!'

    # Application Error Messages
    MSG_REVENUE_RANGES_LOAD_FAILED = 'Revenue Ranges load failed!'
    MSG_REVENUE_RANGE_LOAD_FAILED = 'Revenue Range load failed!'
    MSG_REVENUE_RANGE_INSERT_FAILED = 'Revenue Range insert failed!'
    MSG_REVENUE_RANGE_UPDATE_FAILED = 'Revenue Range update failed!'
    MSG_REVENUE_RANGE_DELETE_FAILED = 'Revenue Range delete failed!'

    # Application Info Messages
    MSG_REVENUE_RANGES_LOAD_SUCCESS = 'Revenue Ranges load successful!'
    MSG_REVENUE_RANGE_LOAD_SUCCESS = 'Revenue Range load successful!'
    MSG_REVENUE_RANGE_INSERT_SUCCESS = 'Revenue Range insert successful!'
    MSG_REVENUE_RANGE_UPDATE_SUCCESS = 'Revenue Range update successful!'
    MSG_REVENUE_RANGE_DELETE_SUCCESS = 'Revenue Range delete successful!'
