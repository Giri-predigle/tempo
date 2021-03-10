from sqlalchemy import Column, Integer, String

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COUserRole(COBase, ORMBase):
    __tablename__ = "tbl_byonic_roles"

    __identifier = Column('user_role_id', Integer, primary_key=True)
    __role = Column('user_role', String)

    __user = None
    __role_description = None

    def __init__(self, identifier, role):
        super().__init__()
        self.identifier = identifier
        self.role = role

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COUserRole.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COUserRole.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.__identifier

    @identifier.setter
    def identifier(self, identifier):
        self.__identifier = identifier

    @property
    def user(self):
        return self.__user

    @user.setter
    def user(self, value):
        self.__user = value

    @property
    def role_description(self):
        return self.__role_description

    @role_description.setter
    def role_description(self, value):
        self.__role_description = value

    @property
    def role(self):
        return self.__role

    @role.setter
    def role(self, role):
        self.__role = role

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.role
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'role': self.role,
                'user': self.user,
                'role_description': self.role_description
                }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        role = json_data['role']
        user_role = COUserRole(identifier=identifier,
                               role=role
                               )
        return user_role

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'role' not in json_data:
            return False
        return True


class COMLUserRole(COMessageLiteral):
    # Validation Error Messages
    MSG_USER_ROLES_NOT_VALID = 'User Roles are not valid!'
    MSG_USER_ROLES_NOT_FOUND = 'User Roles are not found!'
    MSG_USER_ROLE_NOT_VALID = 'User Role is not valid!'
    MSG_USER_ROLE_NOT_FOUND = 'User Role is not found!'

    # Application Error Messages
    MSG_USER_ROLES_LOAD_FAILED = 'User Roles load failed!'
    MSG_USER_ROLE_LOAD_FAILED = 'User Role load failed!'
    MSG_USER_ROLE_INSERT_FAILED = 'User Role insert failed!'
    MSG_USER_ROLE_UPDATE_FAILED = 'User Role update failed!'
    MSG_USER_ROLE_DELETE_FAILED = 'User Role delete failed!'

    # Application Info Messages
    MSG_USER_ROLES_LOAD_SUCCESS = 'User Roles load successful!'
    MSG_USER_ROLE_LOAD_SUCCESS = 'User Role load successful!'
    MSG_USER_ROLE_INSERT_SUCCESS = 'User Role insert successful!'
    MSG_USER_ROLE_UPDATE_SUCCESS = 'User Role update successful!'
    MSG_USER_ROLE_DELETE_SUCCESS = 'User Role delete successful!'
