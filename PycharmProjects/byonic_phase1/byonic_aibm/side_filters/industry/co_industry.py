from sqlalchemy import Column, String, Integer

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COIndustry(COBase, ORMBase):
    __tablename__ = "tbl_industry"

    col_id = Column('industry_id', Integer, primary_key=True)
    __industry_name = Column('industry_name', String)

    def __init__(self, identifier, industry_name):
        super().__init__()
        self.identifier = identifier
        self.industry_name = industry_name

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COIndustry.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COIndustry.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.col_id

    @identifier.setter
    def identifier(self, identifier):
        self.col_id = identifier

    @property
    def industry_name(self):
        return self.__industry_name

    @industry_name.setter
    def industry_name(self, industry_name):
        self.__industry_name = industry_name

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.Industry_code
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'industry_name': self.industry_name}

    @property
    def serialize_by_name(self):
        return {
                'industry_name': self.industry_name
               }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        industry_name = json_data['industry_name']
        industries = COIndustry(identifier=identifier, industry_name=industry_name)
        return industries

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'industry_name' not in json_data:
            return False
        return True


class COMLIndustry(COMessageLiteral):
    # Validation Error Messages
    MSG_INDUSTRIES_NOT_VALID = 'Industries are not valid!'
    MSG_INDUSTRIES_NOT_FOUND = 'Industries are not found!'
    MSG_INDUSTRY_NOT_VALID = 'Industry is not valid!'
    MSG_INDUSTRY_NOT_FOUND = 'Industry is not found!'

    # Application Error Messages
    MSG_INDUSTRIES_LOAD_FAILED = 'Industries load failed!'
    MSG_INDUSTRY_LOAD_FAILED = 'Industry load failed!'
    MSG_INDUSTRY_INSERT_FAILED = 'Industry insert failed!'
    MSG_INDUSTRY_UPDATE_FAILED = 'Industry update failed!'
    MSG_INDUSTRY_DELETE_FAILED = 'Industry delete failed!'

    # Application Info Messages
    MSG_INDUSTRIES_LOAD_SUCCESS = 'Industries load successful!'
    MSG_INDUSTRY_LOAD_SUCCESS = 'Industry load successful!'
    MSG_INDUSTRY_INSERT_SUCCESS = 'Industry insert successful!'
    MSG_INDUSTRY_UPDATE_SUCCESS = 'Industry update successful!'
    MSG_INDUSTRY_DELETE_SUCCESS = 'Industry delete successful!'
