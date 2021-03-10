from byonic_aibm.side_filters.employee_size.co_employee_size import COEmployee, COMLEmployee

from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral


class DAEmployee(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DAEmployee.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DAEmployee.__logger

    def load_all(self):
        dbsession = None
        employee_size = dict()
        try:
            dbsession = DBConnection().getsession()
            employee_size_list = dbsession.query(COEmployee).all()

            if employee_size_list is None:
                message = COMLEmployee.MSG_EMPLOYEE_VALUES_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            for ct in employee_size_list:
                employee_size[ct.get_key()] = ct

            self.get_logger().info(COMLEmployee.MSG_EMPLOYEE_VALUES_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLEmployee.MSG_EMPLOYEE_VALUES_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return employee_size

    def load_employee_size(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            employee_size: COEmployee = dbsession.query(COEmployee).get(identifier)

            if employee_size is None:
                message = COMLEmployee.MSG_EMPLOYEE_VALUE_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLEmployee.MSG_EMPLOYEE_VALUE_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    employee_size.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return employee_size

    def add_employee_size(self, employee_size: COEmployee):
        dbsession = None
        try:

            if not isinstance(employee_size, COEmployee):
                message = COMLEmployee.MSG_EMPLOYEE_VALUE_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    employee_size.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            dbsession.add(employee_size)
            dbsession.commit()

            self.get_logger().info(
                COMLEmployee.MSG_EMPLOYEE_VALUE_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    employee_size.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                employee_size.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_employee_size(self, employee_size: COEmployee):
        dbsession = None
        try:

            if not isinstance(employee_size, COEmployee):
                message = COMLEmployee.MSG_EMPLOYEE_VALUE_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    employee_size.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_employee_size: COEmployee = dbsession.query(COEmployee).get(employee_size.identifier)
            db_employee_size.identifier = employee_size.identifier
            db_employee_size.name = employee_size.name

            dbsession.commit()

            self.get_logger().info(
                COMLEmployee.MSG_EMPLOYEE_VALUE_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    employee_size.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                employee_size.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_employee_size(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_employee_size: COEmployee = dbsession.query(COEmployee).get(identifier)
            dbsession.delete(db_employee_size)
            dbsession.commit()

            self.get_logger().info(
                COMLEmployee.MSG_EMPLOYEE_VALUE_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
