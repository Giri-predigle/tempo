from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_aibm.usermanagement.user_role.co_user_role import COUserRole, COMLUserRole
from byonic_object.co_literal import COStringLiteral


class DAUserRole(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DAUserRole.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DAUserRole.__logger

    def load_all(self, per_page: int):
        dbsession = None
        users = dict()
        filters = dict()
        try:
            dbsession = DBConnection().getsession()
            user_list = dbsession.query(COUserRole).limit(per_page).all()

            if user_list is None:
                message = COMLUserRole.MSG_USER_ROLES_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            for ct in user_list:
                users[ct.get_key()] = ct

            self.get_logger().info(COMLUserRole.MSG_USER_ROLES_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLUserRole.MSG_USER_ROLES_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return users

    def load_user_role(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            user_role: COUserRole = dbsession.query(COUserRole).get(identifier)

            if user_role is None:
                message = COMLUserRole.MSG_USER_ROLE_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLUserRole.MSG_USER_ROLE_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    user_role.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLUserRole.MSG_USER_ROLE_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return user_role

    def add_user_role(self, user_role: COUserRole):
        dbsession = None
        try:

            if not isinstance(user_role, COUserRole):
                message = COMLUserRole.MSG_USER_ROLE_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    user_role.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            # encrypt the password
            # user.password = generate_password_hash(user.password)

            dbsession = DBConnection().getsession()
            dbsession.add(user_role)
            dbsession.commit()

            self.get_logger().info(
                COMLUserRole.MSG_USER_ROLE_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    user_role.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLUserRole.MSG_USER_ROLE_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                user_role.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_user_role(self, user_role: COUserRole):
        dbsession = None
        try:

            if not isinstance(user_role, COUserRole):
                message = COMLUserRole.MSG_USER_ROLE_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    user_role.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            # encrypt the password
            # user.password = generate_password_hash(user.password)

            dbsession = DBConnection().getsession()
            db_user_role: COUserRole = dbsession.query(COUserRole).get(user_role.identifier)
            db_user_role.identifier = user_role.identifier
            db_user_role.role = user_role.role

            dbsession.commit()

            self.get_logger().info(
                COMLUserRole.MSG_USER_ROLE_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    user_role.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLUserRole.MSG_USER_ROLE_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                user_role.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_user_role(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_user_role: COUserRole = dbsession.query(COUserRole).get(identifier)
            dbsession.delete(db_user_role)
            dbsession.commit()

            self.get_logger().info(
                COMLUserRole.MSG_USER_ROLE_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLUserRole.MSG_USER_ROLE_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
