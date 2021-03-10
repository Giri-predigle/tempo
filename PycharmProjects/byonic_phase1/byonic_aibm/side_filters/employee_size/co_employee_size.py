from sqlalchemy import Column, String, Integer

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COEmployee(COBase, ORMBase):
    __tablename__ = "tbl_employee_size_range"

    col_id = Column('employee_size_range_id', Integer, primary_key=True)
    __employee_values = Column('employee_range_values', String)

    def __init__(self, identifier, employee_values):
        super().__init__()
        self.identifier = identifier
        self.employee_values = employee_values

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COEmployee.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COEmployee.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.col_id

    @identifier.setter
    def identifier(self, identifier):
        self.col_id = identifier

    @property
    def employee_values(self):
        return self.__employee_values

    @employee_values.setter
    def employee_values(self, employee_values):
        self.__employee_values = employee_values

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.employee_values,
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'employee_values': self.employee_values
                }

    @property
    def serialize_by_value(self):
        return {
                'employee_values': self.employee_values
                }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        employee_values = json_data['employee_values']
        employee_ranges = COEmployee(identifier=identifier, employee_values=employee_values)
        return employee_ranges

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'employee_values' not in json_data:
            return False
        return True


class COMLEmployee(COMessageLiteral):
    # Validation Error Messages
    MSG_EMPLOYEE_VALUES_NOT_VALID = 'Employee Values are not valid!'
    MSG_EMPLOYEE_VALUES_NOT_FOUND = 'EMPLOYEES VALUES are not found!'
    MSG_EMPLOYEE_VALUE_NOT_VALID = 'Employee Value is not valid!'
    MSG_EMPLOYEE_VALUE_NOT_FOUND = 'Employee Value is not found!'

    # Application Error Messages
    MSG_EMPLOYEE_VALUES_LOAD_FAILED = 'Employee Values load failed!'
    MSG_EMPLOYEE_VALUE_LOAD_FAILED = 'Employee Value load failed!'
    MSG_EMPLOYEE_VALUE_INSERT_FAILED = 'Employee Value insert failed!'
    MSG_EMPLOYEE_VALUE_UPDATE_FAILED = 'Employee Value update failed!'
    MSG_EMPLOYEE_VALUE_DELETE_FAILED = 'Employee Value delete failed!'

    # Application Info Messages
    MSG_EMPLOYEE_VALUES_LOAD_SUCCESS = 'Employee Values load successful!'
    MSG_EMPLOYEE_VALUE_LOAD_SUCCESS = 'Employee Value load successful!'
    MSG_EMPLOYEE_VALUE_INSERT_SUCCESS = 'Employee Value insert successful!'
    MSG_EMPLOYEE_VALUE_UPDATE_SUCCESS = 'Employee Value update successful!'
    MSG_EMPLOYEE_VALUE_DELETE_SUCCESS = 'Employee Value delete successful!'
