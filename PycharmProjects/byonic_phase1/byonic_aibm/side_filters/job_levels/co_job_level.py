from sqlalchemy import Column, String, Integer

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COJobLevel(COBase, ORMBase):
    __tablename__ = "tbl_job_level"

    col_id = Column('job_level_id', Integer, primary_key=True)
    col_name = Column('job_level_name', String, nullable=False)

    def __init__(self, identifier, name):
        super().__init__()
        self.identifier = identifier
        self.name = name

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COJobLevel.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COJobLevel.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.col_id

    @identifier.setter
    def identifier(self, identifier):
        self.col_id = identifier

    @property
    def name(self):
        return self.col_name

    @name.setter
    def name(self, name):
        self.col_name = name

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.name,
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'name': self.name
                }

    @property
    def serialize_by_name(self):
        return {
                'name': self.name
                }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        name = json_data['name']
        employee_ranges = COJobLevel(identifier=identifier, name=name)
        return employee_ranges

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'name' not in json_data:
            return False
        return True


class COMLJobLevel(COMessageLiteral):
    # Validation Error Messages
    MSG_JOB_LEVELS_NOT_VALID = 'Job Levels are not valid!'
    MSG_JOB_LEVELS_NOT_FOUND = 'Job Levels are not found!'
    MSG_JOB_LEVEL_NOT_VALID = 'Job Level is not valid!'
    MSG_JOB_LEVEL_NOT_FOUND = 'Job Level is not found!'

    # Application Error Messages
    MSG_JOB_LEVELS_LOAD_FAILED = 'Job Levels load failed!'
    MSG_JOB_LEVEL_LOAD_FAILED = 'Job Level load failed!'
    MSG_JOB_LEVEL_INSERT_FAILED = 'Job Level insert failed!'
    MSG_JOB_LEVEL_UPDATE_FAILED = 'Job Level update failed!'
    MSG_JOB_LEVEL_DELETE_FAILED = 'Job Level delete failed!'

    # Application Info Messages
    MSG_JOB_LEVELS_LOAD_SUCCESS = 'Job Levels load successful!'
    MSG_JOB_LEVEL_LOAD_SUCCESS = 'Job Level load successful!'
    MSG_JOB_LEVEL_INSERT_SUCCESS = 'Job Level insert successful!'
    MSG_JOB_LEVEL_UPDATE_SUCCESS = 'Job Level update successful!'
    MSG_JOB_LEVEL_DELETE_SUCCESS = 'Job Level delete successful!'
