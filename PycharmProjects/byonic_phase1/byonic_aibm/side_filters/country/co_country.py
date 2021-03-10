from sqlalchemy import Column, String, Integer

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COCountry(COBase, ORMBase):
    __tablename__ = "tbl_country"

    col_id = Column('country_id', Integer, primary_key=True)
    col_code = Column('country_code', String)
    col_name = Column('country_name', String)
    col_zone = Column('zone', String)

    def __init__(self, identifier, country_code, country_name, zone):
        super().__init__()
        self.identifier = identifier
        self.country_code = country_code
        self.country_name = country_name
        self.zone = zone

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COCountry.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COCountry.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.col_id

    @identifier.setter
    def identifier(self, identifier):
        self.col_id = identifier

    @property
    def country_code(self):
        return self.col_code

    @country_code.setter
    def country_code(self, country_code):
        self.col_code = country_code

    @property
    def zone(self):
        return self.col_zone

    @zone.setter
    def zone(self, value):
        self._zone = value

    @property
    def country_name(self):
        return self.col_name

    @country_name.setter
    def country_name(self, country_name):
        self.col_name = country_name

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.country_code,
                self.country_name,
                self.zone,
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'country_code': self.country_code,
                'country_name': self.country_name,
                'zone': self.zone,
                }

    @property
    def serialize_by_name(self):
        return {
                'country_name': self.country_name
               }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        country_code = json_data['country_code']
        country_name = json_data['country_name']
        zone = json_data['zone']
        countries = COCountry(identifier=identifier, country_code=country_code,
                              country_name=country_name, zone=zone)
        return countries

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'country_code' not in json_data:
            return False
        return True


class COMLCountry(COMessageLiteral):
    # Validation Error Messages
    MSG_COUNTRIES_NOT_VALID = 'Countries are not valid!'
    MSG_COUNTRIES_NOT_FOUND = 'Countries are not found!'
    MSG_COUNTRY_NOT_VALID = 'Country is not valid!'
    MSG_COUNTRY_NOT_FOUND = 'Country is not found!'

    # Application Error Messages
    MSG_COUNTRIES_LOAD_FAILED = 'Countries load failed!'
    MSG_COUNTRY_LOAD_FAILED = 'Country load failed!'
    MSG_COUNTRY_INSERT_FAILED = 'Country insert failed!'
    MSG_COUNTRY_UPDATE_FAILED = 'Country update failed!'
    MSG_COUNTRY_DELETE_FAILED = 'Country delete failed!'

    # Application Info Messages
    MSG_COUNTRIES_LOAD_SUCCESS = 'Countries load successful!'
    MSG_COUNTRY_LOAD_SUCCESS = 'Country load successful!'
    MSG_COUNTRY_INSERT_SUCCESS = 'Country insert successful!'
    MSG_COUNTRY_UPDATE_SUCCESS = 'Country update successful!'
    MSG_COUNTRY_DELETE_SUCCESS = 'Country delete successful!'
