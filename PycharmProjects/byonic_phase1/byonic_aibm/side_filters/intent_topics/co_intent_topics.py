from sqlalchemy import Column, String, Integer, Date, Enum

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COTopic(COBase, ORMBase):
    __tablename__ = "tbl_topic"

    col_id = Column('topic_id', Integer, primary_key=True)
    col_name = Column('topic_name', String, nullable=False)

    def __init__(self, identifier, name):
        super().__init__()
        self.identifier = identifier
        self.name = name

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COTopic.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COTopic.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

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
        employee_ranges = COTopic(identifier=identifier, name=name)
        return employee_ranges

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'name' not in json_data:
            return False
        return True


class COMLTopic(COMessageLiteral):
    # Validation Error Messages
    MSG_TOPICS_NOT_VALID = 'Topics are not valid!'
    MSG_TOPICS_NOT_FOUND = 'Topics are not found!'
    MSG_TOPIC_NOT_VALID = 'Topic is not valid!'
    MSG_TOPIC_NOT_FOUND = 'Topic is not found!'

    # Application Error Messages
    MSG_TOPICS_LOAD_FAILED = 'Topics load failed!'
    MSG_TOPIC_LOAD_FAILED = 'Topic load failed!'
    MSG_TOPIC_INSERT_FAILED = 'Topic insert failed!'
    MSG_TOPIC_UPDATE_FAILED = 'Topic update failed!'
    MSG_TOPIC_DELETE_FAILED = 'Topic delete failed!'

    # Application Info Messages
    MSG_TOPICS_LOAD_SUCCESS = 'Topics load successful!'
    MSG_TOPIC_LOAD_SUCCESS = 'Topic load successful!'
    MSG_TOPIC_INSERT_SUCCESS = 'Topic insert successful!'
    MSG_TOPIC_UPDATE_SUCCESS = 'Topic update successful!'
    MSG_TOPIC_DELETE_SUCCESS = 'Topic delete successful!'
