from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_aibm.usermanagement.user_role.co_user_role import COMLUserRole, COUserRole
from byonic_aibm.usermanagement.user_role.da_user_role import DAUserRole


class BLUserRole(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLUserRole.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_user_role: DAUserRole = DAUserRole()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLUserRole.__logger

    def get_user_roles(self, per_page):
        user_roles = self.da_user_role.load_all(per_page)
        if user_roles is not None:
            for user_role_key in user_roles.keys():
                user_role = user_roles[user_role_key]
                self.cache.put(user_role_key, user_role)
        return user_roles

    def get_user_role(self, identifier):
        user_role_key = COUserRole.create_key(identifier=identifier)
        user_role = self.cache.get(user_role_key)
        if user_role is None:
            user_role = self.da_user_role.load_user_role(identifier)
            self.validate_user_role(user_role)
            self.cache.put(user_role.get_key(), user_role)
        return user_role

    def create_user_role(self, user_role):
        self.validate_user_role(user_role)
        self.da_user_role.add_user_role(user_role)
        if user_role.identifier is None:
            message = COMLUserRole.MSG_USER_ROLE_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return user_role

    def update_user_role(self, user_role):
        self.validate_user_role(user_role)
        self.da_user_role.update_user_role(user_role)
        self.cache.remove(user_role.get_key())
        return user_role

    def delete_user_role(self, identifier):
        self.da_user_role.delete_user_role(identifier)
        user_role_key = COUserRole.create_key(identifier=identifier)
        self.cache.remove(user_role_key)
        return identifier

    def save_user_role(self, user_role):
        self.validate_user_role(user_role)
        if user_role.mark_for_deletion:
            self.delete_user_role(user_role.identifier)
        elif user_role.touched:
            if user_role.identifier is None:
                self.create_user_role(user_role)
            else:
                self.update_user_role(user_role)
            user_role.touched = False

    def validate_user_role(self, user_role):
        if not isinstance(user_role, COUserRole):
            message = COMLUserRole.MSG_USER_ROLE_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
