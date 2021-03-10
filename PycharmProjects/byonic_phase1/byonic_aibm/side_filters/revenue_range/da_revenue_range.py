from byonic_aibm.side_filters.revenue_range.co_revenue_range import CORevenue
from byonic_aibm.side_filters.revenue_range.co_revenue_range import COMLRevenue
from scripts.byonic_data_access.db_connection_config import DBConnection

from byonic_core.fw_dataaccess import DABase
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral


class DARevenue(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DARevenue.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DARevenue.__logger

    def load_all(self):
        dbsession = None
        revenue_range_dict = dict()
        try:
            dbsession = DBConnection().getsession()

            revenue_range_list = dbsession.query(CORevenue).all()

            if revenue_range_list is None:
                message = COMLRevenue.MSG_REVENUE_RANGES_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            try:
                for ct in revenue_range_list:
                    for cts in ct:
                        revenue_range_dict[cts.get_key()] = cts

            except TypeError:
                for ct in revenue_range_list:
                    revenue_range_dict[ct.get_key()] = ct

            self.get_logger().info(COMLRevenue.MSG_REVENUE_RANGES_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLRevenue.MSG_REVENUE_RANGES_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return revenue_range_dict

    def load_revenue_range(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            revenue_range: CORevenue = dbsession.query(CORevenue).get(identifier)

            if revenue_range is None:
                message = COMLRevenue.MSG_REVENUE_RANGE_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLRevenue.MSG_REVENUE_RANGE_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLRevenue.MSG_REVENUE_RANGE_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return revenue_range
    
    def add_revenue_range(self, revenue_range: CORevenue):
        dbsession = None
        try:

            if not isinstance(revenue_range, CORevenue):
                message = COMLRevenue.MSG_REVENUE_RANGE_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            dbsession.add(revenue_range)
            dbsession.commit()

            self.get_logger().info(
                COMLRevenue.MSG_REVENUE_RANGE_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLRevenue.MSG_REVENUE_RANGE_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_revenue_range(self, revenue_range: CORevenue):
        dbsession = None
        try:

            if not isinstance(revenue_range, CORevenue):
                message = COMLRevenue.MSG_REVENUE_RANGE_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_revenue_range: CORevenue = dbsession.query(CORevenue).get(revenue_range.identifier)
            db_revenue_range.identifier = revenue_range.identifier
            db_revenue_range.name = revenue_range.name

            dbsession.commit()

            self.get_logger().info(
                COMLRevenue.MSG_REVENUE_RANGE_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLRevenue.MSG_REVENUE_RANGE_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    revenue_range.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_revenue_range(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_revenue_range: CORevenue = dbsession.query(CORevenue).get(identifier)
            dbsession.delete(db_revenue_range)
            dbsession.commit()

            self.get_logger().info(
                COMLRevenue.MSG_REVENUE_RANGE_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLRevenue.MSG_REVENUE_RANGE_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
