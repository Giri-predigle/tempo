from sqlalchemy import Column, String, Integer, Enum, Date

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COByonicUser(COBase, ORMBase):
    __tablename__ = "tbl_byonic_user"

    __identifier = Column('user_id', Integer, primary_key=True)
    __user_first_name = Column('user_first_name', String)
    __user_last_name = Column('user_last_name', String)
    col_mail = Column('user_email', String)
    col_password = Column('user_password', String)
    __role_id = Column('user_role_id', Integer)
    __status = Column('Status', Enum('Active', 'InActive'))
    __reset = Column('reset', Enum('0', '1'))
    col_org_id = Column('user_organisation_id', Integer)
    __last_login = Column('last_login', Date)

    def __init__(self, identifier, user_first_name, user_last_name, email, reset, status,
                 password, organisation_id, role_id):
        super().__init__()
        self.identifier = identifier
        self.user_first_name = user_first_name
        self.user_last_name = user_last_name
        self.email = email
        self.reset = reset
        self.password = password
        self.role_id = role_id
        self.organisation_id = organisation_id
        self.status = status

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COByonicUser.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COByonicUser.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.__identifier

    @identifier.setter
    def identifier(self, identifier):
        self.__identifier = identifier

    @property
    def user_first_name(self):
        return self.__user_first_name

    @user_first_name.setter
    def user_first_name(self, user_first_name):
        self.__user_first_name = user_first_name

    @property
    def user_last_name(self):
        return self.__user_last_name

    @user_last_name.setter
    def user_last_name(self, user_last_name):
        self.__user_last_name = user_last_name

    @property
    def email(self):
        return self.col_mail

    @email.setter
    def email(self, email):
        self.col_mail = email

    @property
    def reset(self):
        return self.__reset

    @reset.setter
    def reset(self, reset):
        self.__reset = reset

    @property
    def status(self):
        return self.__status

    @status.setter
    def status(self, status):
        self.__status = status

    @property
    def organisation_id(self):
        return self.col_org_id

    @organisation_id.setter
    def organisation_id(self, value):
        self.col_org_id = value

    @property
    def role_id(self):
        return self.__role_id

    @role_id.setter
    def role_id(self, value):
        self.__role_id = value

    @property
    def password(self):
        return self.col_password

    @password.setter
    def password(self, value):
        self.col_password = value

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.user_first_name,
                self.user_last_name,
                self.email,
                self.reset,
                self.status,
                self.organisation_id,
                self.role_id,
                self.password
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'id': self.identifier,
                'user_first_name': self.user_first_name,
                'user_last_name': self.user_last_name,
                'email': self.email,
                'reset': self.reset,
                'status': self.status,
                'role_id': self.role_id,
                'org_id': self.organisation_id,
                'password': self.password,
                }

    @property
    def serialize_login(self):
        return {'id': self.identifier,
                'reset': self.reset,
                'org_id': self.org,
                'role_id': self.role_id}

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'id' in json_data:
            identifier = json_data['id']
        user_first_name = json_data['user_first_name']
        user_last_name = json_data['user_last_name']
        email = json_data['email']
        reset = json_data['reset']
        status = json_data['status']
        organisation_id = json_data['org_id']
        role_id = json_data['role_id']
        password = json_data['password']
        byonic_user = COByonicUser(identifier=identifier,
                                   user_first_name=user_first_name,
                                   user_last_name=user_last_name,
                                   email=email,
                                   reset=reset,
                                   status=status,
                                   organisation_id=organisation_id,
                                   password=password,
                                   role_id=role_id
                                   )
        return byonic_user

    @classmethod
    def validate_json(cls, json_data):
        if 'email' not in json_data:
            return False
        if 'password' not in json_data:
            return False
        return True


class COMLByonicUser(COMessageLiteral):
    # Validation Error Messages
    MSG_BYONIC_USERS_NOT_VALID = 'Byonic users are not valid!'
    MSG_BYONIC_USERS_NOT_FOUND = 'Byonic users are not found!'
    MSG_BYONIC_USER_NOT_VALID = 'Byonic user is not valid!'
    MSG_BYONIC_USER_NOT_FOUND = 'Byonic user is not found!'

    # Application Error Messages
    MSG_BYONIC_USERS_LOAD_FAILED = 'Byonic users load failed!'
    MSG_BYONIC_USER_LOAD_FAILED = 'Byonic user load failed!'
    MSG_BYONIC_USER_INSERT_FAILED = 'Byonic user insert failed!'
    MSG_BYONIC_USER_UPDATE_FAILED = 'Byonic user update failed!'
    MSG_BYONIC_USER_DELETE_FAILED = 'Byonic user delete failed!'

    # Application Info Messages
    MSG_BYONIC_USERS_LOAD_SUCCESS = 'Byonic users load successful!'
    MSG_BYONIC_USER_LOAD_SUCCESS = 'Byonic user load successful!'
    MSG_BYONIC_USER_INSERT_SUCCESS = 'Byonic user insert successful!'
    MSG_BYONIC_USER_UPDATE_SUCCESS = 'Byonic user update successful!'
    MSG_BYONIC_USER_DELETE_SUCCESS = 'Byonic user delete successful!'
