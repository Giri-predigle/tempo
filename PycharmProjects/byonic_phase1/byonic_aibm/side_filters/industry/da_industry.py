from byonic_aibm.side_filters.industry.co_industry import COIndustry
from byonic_aibm.side_filters.industry.co_industry import COMLIndustry

from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral


class DAIndustry(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DAIndustry.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DAIndustry.__logger

    def load_all(self):
        dbsession = None
        industry_dict = dict()
        try:
            dbsession = DBConnection().getsession()

            industry_list = dbsession.query(COIndustry).all()

            if industry_list is None:
                message = COMLIndustry.MSG_INDUSTRIES_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            try:
                for ct in industry_list:
                    for cts in ct:
                        industry_dict[cts.get_key()] = cts
            except TypeError:
                for ct in industry_list:
                    industry_dict[ct.get_key()] = ct

            self.get_logger().info(COMLIndustry.MSG_INDUSTRIES_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLIndustry.MSG_INDUSTRIES_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return industry_dict

    def load_industry(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            industry: COIndustry = dbsession.query(COIndustry).get(identifier)

            if industry is None:
                message = COMLIndustry.MSG_INDUSTRY_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLIndustry.MSG_INDUSTRY_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLIndustry.MSG_INDUSTRY_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return industry
    
    def add_industry(self, industry: COIndustry):
        dbsession = None
        try:

            if not isinstance(industry, COIndustry):
                message = COMLIndustry.MSG_INDUSTRY_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            dbsession.add(industry)
            dbsession.commit()

            self.get_logger().info(
                COMLIndustry.MSG_INDUSTRY_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLIndustry.MSG_INDUSTRY_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_industry(self, industry: COIndustry):
        dbsession = None
        try:

            if not isinstance(industry, COIndustry):
                message = COMLIndustry.MSG_INDUSTRY_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_industry: COIndustry = dbsession.query(COIndustry).get(industry.identifier)
            db_industry.identifier = industry.identifier
            db_industry.name = industry.name

            dbsession.commit()

            self.get_logger().info(
                COMLIndustry.MSG_INDUSTRY_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLIndustry.MSG_INDUSTRY_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    industry.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_industry(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_industry: COIndustry = dbsession.query(COIndustry).get(identifier)
            dbsession.delete(db_industry)
            dbsession.commit()

            self.get_logger().info(
                COMLIndustry.MSG_INDUSTRY_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLIndustry.MSG_INDUSTRY_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
