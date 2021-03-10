from sqlalchemy import Column, Integer, String

from byonic_object.co_base import COBase, ORMBase
from byonic_object.co_literal import COMessageLiteral
from byonic_object.co_literal import COStringLiteral


class COOrganization(COBase, ORMBase):
    __tablename__ = "tbl_byonic_organisation"

    col_user_organisation_id = Column('user_organisation_id', Integer, primary_key=True)
    __organization = Column('user_organisation', String)

    def __init__(self, identifier, organization):
        super().__init__()
        self.identifier = identifier
        self.organization = organization

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COOrganization.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COOrganization.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.col_user_organisation_id

    @identifier.setter
    def identifier(self, identifier):
        self.col_user_organisation_id = identifier

    @property
    def organization(self):
        return self.__organization

    @organization.setter
    def organization(self, organization):
        self.__organization = organization

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecessary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.identifier,
                self.organization
            )
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifier': self.identifier,
                'organization': self.organization
                }

    @classmethod
    def deserialize(cls, json_data):
        identifier = None
        if 'identifier' in json_data:
            identifier = json_data['identifier']
        organization = json_data['organization']
        organization = COOrganization(identifier=identifier,
                                      organization=organization
                                      )
        return organization

    @classmethod
    def validate_json(cls, json_data):
        if 'identifier' not in json_data:
            return False
        if 'organization' not in json_data:
            return False
        return True


class COMLOrganization(COMessageLiteral):
    # Validation Error Messages
    MSG_ORGANIZATIONS_NOT_VALID = 'Organizations are not valid!'
    MSG_ORGANIZATIONS_NOT_FOUND = 'Organizations are not found!'
    MSG_ORGANIZATION_NOT_VALID = 'Organization is not valid!'
    MSG_ORGANIZATION_NOT_FOUND = 'Organization is not found!'

    # Application Error Messages
    MSG_ORGANIZATIONS_LOAD_FAILED = 'Organizations load failed!'
    MSG_ORGANIZATION_LOAD_FAILED = 'Organization load failed!'
    MSG_ORGANIZATION_INSERT_FAILED = 'Organization insert failed!'
    MSG_ORGANIZATION_UPDATE_FAILED = 'Organization update failed!'
    MSG_ORGANIZATION_DELETE_FAILED = 'Organization delete failed!'

    # Application Info Messages
    MSG_ORGANIZATIONS_LOAD_SUCCESS = 'Organizations load successful!'
    MSG_ORGANIZATION_LOAD_SUCCESS = 'Organization load successful!'
    MSG_ORGANIZATION_INSERT_SUCCESS = 'Organization insert successful!'
    MSG_ORGANIZATION_UPDATE_SUCCESS = 'Organization update successful!'
    MSG_ORGANIZATION_DELETE_SUCCESS = 'Organization delete successful!'
