import hashlib
from byonic_aibm.usermanagement.byonic_user.co_byonic_user import COByonicUser
from byonic_aibm.usermanagement.byonic_user.co_byonic_user import COMLByonicUser

from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral


class DAByonicUser(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DAByonicUser.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DAByonicUser.__logger

    def load_all(self):
        dbsession = None
        byonic_users = dict()
        try:
            dbsession = DBConnection().getsession()
            byonic_user_list = dbsession.query(COByonicUser).all()
            if byonic_user_list is None:
                message = COMLByonicUser.MSG_BYONIC_USERS_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            for ct in byonic_user_list:
                byonic_users[ct.get_key()] = ct

            self.get_logger().info(COMLByonicUser.MSG_BYONIC_USERS_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLByonicUser.MSG_BYONIC_USERS_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return byonic_users

    def login_user(self, mail: str, password: str):
        dbsession = None
        byonic_users = dict()
        try:
            dbsession = DBConnection().getsession()
            password = hashlib.sha512(password.encode("utf-8")).hexdigest()
            byonic_user_list = dbsession.query(COByonicUser).filter(COByonicUser.col_mail
                                                                    == mail). \
                filter(COByonicUser.col_password == password).all()

            if byonic_user_list is None:
                message = COMLByonicUser.MSG_BYONIC_USERS_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            for ct in byonic_user_list:
                byonic_users[ct.get_key()] = ct

            self.get_logger().info(COMLByonicUser.MSG_BYONIC_USERS_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLByonicUser.MSG_BYONIC_USERS_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return byonic_users

    def filter_user(self, org_id: int):
        dbsession = None
        global user_list
        byonic_users = dict()
        try:
            dbsession = DBConnection().getsession()
            byonic_user_list = dbsession.query(COByonicUser)
            print(type(org_id))
            if org_id == str(1):
                byonic_user_list = byonic_user_list

            else:
                byonic_user_list = byonic_user_list.filter(COByonicUser.col_org_id == org_id)

            byonic_user_list = byonic_user_list.all()

            if byonic_user_list is None:
                message = COMLByonicUser.MSG_BYONIC_USERS_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)
            print(byonic_user_list)
            for ct in byonic_user_list:
                byonic_users[ct.get_key()] = ct

            self.get_logger().info(COMLByonicUser.MSG_BYONIC_USERS_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLByonicUser.MSG_BYONIC_USERS_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return byonic_users

    def load_byonic_user(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            byonic_user: COByonicUser = dbsession.query(COByonicUser).get(identifier)

            if byonic_user is None:
                message = COMLByonicUser.MSG_BYONIC_USER_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLByonicUser.MSG_BYONIC_USER_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    byonic_user.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLByonicUser.MSG_BYONIC_USER_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return byonic_user

    def add_byonic_user(self, byonic_user: COByonicUser):
        dbsession = None
        try:

            if not isinstance(byonic_user, COByonicUser):
                message = COMLByonicUser.MSG_BYONIC_USER_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    byonic_user.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            byonic_user.password = hashlib.sha512(byonic_user.password.encode("utf-8")).hexdigest()
            dbsession.add(byonic_user)
            dbsession.commit()

            self.get_logger().info(
                COMLByonicUser.MSG_BYONIC_USER_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    byonic_user.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLByonicUser.MSG_BYONIC_USER_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                byonic_user.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_byonic_user(self, byonic_user: COByonicUser):
        dbsession = None
        try:

            if not isinstance(byonic_user, COByonicUser):
                message = COMLByonicUser.MSG_BYONIC_USER_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    byonic_user.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_byonic_user: COByonicUser = dbsession.query(COByonicUser).get(byonic_user.identifier)
            db_byonic_user.identifier = byonic_user.identifier
            db_byonic_user.reset = byonic_user.reset
            db_byonic_user.user_first_name = byonic_user.user_first_name
            db_byonic_user.user_last_name = byonic_user.user_last_name
            db_byonic_user.email = byonic_user.email
            db_byonic_user.password = hashlib.sha512(byonic_user.password.encode("utf-8")).hexdigest()
            db_byonic_user.organisation_id = byonic_user.organisation_id
            db_byonic_user.role_id = byonic_user.role_id
            db_byonic_user.status = byonic_user.status

            dbsession.commit()

            self.get_logger().info(
                COMLByonicUser.MSG_BYONIC_USER_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    byonic_user.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLByonicUser.MSG_BYONIC_USER_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                byonic_user.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_byonic_user(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_byonic_user: COByonicUser = dbsession.query(COByonicUser).get(identifier)
            dbsession.delete(db_byonic_user)
            dbsession.commit()

            self.get_logger().info(
                COMLByonicUser.MSG_BYONIC_USER_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLByonicUser.MSG_BYONIC_USER_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
