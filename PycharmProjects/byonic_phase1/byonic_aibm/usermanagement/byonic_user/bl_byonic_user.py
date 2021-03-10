from byonic_aibm.usermanagement.organisation.da_organisation import DAOrganisation
from byonic_aibm.usermanagement.user_role.bl_user_role import BLUserRole
from byonic_aibm.usermanagement.byonic_user.co_byonic_user import COByonicUser, COMLByonicUser
from byonic_aibm.usermanagement.byonic_user.da_byonic_user import DAByonicUser
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLByonicUser(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLByonicUser.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_byonic_user: DAByonicUser = DAByonicUser()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLByonicUser.__logger

    def get_byonic_users(self, org_id):

        byonic_users = self.da_byonic_user.filter_user(org_id=org_id)

        if byonic_users is not None:
            for byonic_user_key in byonic_users.keys():
                byonic_user = byonic_users[byonic_user_key]
                self.get_role_detail(byonic_user)
                self.cache.put(byonic_user_key, byonic_user)
        return byonic_users

    def login_byonic_users(self, password, mail):

        byonic_users = self.da_byonic_user.login_user(password, mail)

        if byonic_users is not None:
            for byonic_user_key in byonic_users.keys():
                byonic_user = byonic_users[byonic_user_key]
                self.get_role_detail(byonic_user)
                self.cache.put(byonic_user_key, byonic_user)
        return byonic_users

    def get_byonic_user(self, identifier):
        byonic_user_key = COByonicUser.create_key(identifier=identifier)
        byonic_user = self.cache.get(byonic_user_key)
        if byonic_user is None:
            byonic_user = self.da_byonic_user.load_byonic_user(identifier)
            self.validate_byonic_user(byonic_user)
            self.get_role_detail(byonic_user)
            # self.cache.put(byonic_user.get_key(), byonic_user)
        return byonic_user

    @staticmethod
    def get_role_detail(byonic_user):
        if byonic_user.role_id is not None:

            role = BLUserRole().get_user_role(byonic_user.role_id)
            byonic_user.role_id = role.serialize

        if byonic_user.organisation_id is not None:

            organization = DAOrganisation().load_organisation(byonic_user.organisation_id)
            for i in organization.values():
                byonic_user.organisation_id = i.serialize

    def create_byonic_user(self, byonic_user):
        self.validate_byonic_user(byonic_user)
        self.da_byonic_user.add_byonic_user(byonic_user)
        if byonic_user.identifier is None:
            message = COMLByonicUser.MSG_BYONIC_USER_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return byonic_user

    def update_byonic_user(self, byonic_user):
        self.validate_byonic_user(byonic_user)
        self.da_byonic_user.update_byonic_user(byonic_user)
        self.cache.remove(byonic_user.get_key())
        return byonic_user

    def delete_byonic_user(self, identifier):
        self.da_byonic_user.delete_byonic_user(identifier)
        byonic_user_key = COByonicUser.create_key(identifier=identifier)
        self.cache.remove(byonic_user_key)
        return identifier

    def save_byonic_users(self, org_id):
        for byonic_user in self.get_byonic_users(org_id=org_id):
            self.save_byonic_user(byonic_user)

    def save_byonic_user(self, byonic_user):
        self.validate_byonic_user(byonic_user)
        if byonic_user.mark_for_deletion:
            self.delete_byonic_user(byonic_user.identifier)
        elif byonic_user.touched:
            if byonic_user.identifier is None:
                self.create_byonic_user(byonic_user)
            else:
                self.update_byonic_user(byonic_user)
            byonic_user.touched = False

    def validate_byonic_user(self, byonic_user):
        if not isinstance(byonic_user, COByonicUser):
            message = COMLByonicUser.MSG_BYONIC_USER_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
