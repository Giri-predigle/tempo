from byonic_aibm.side_filters.country.co_country import COCountry, COMLCountry
from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral

global zones, countries


class DACountry(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DACountry.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DACountry.__logger

    def load_all(self):
        dbsession = None
        country = dict()
        try:
            dbsession = DBConnection().getsession()

            country_list = dbsession.query(COCountry).all()

            if country_list is None:
                message = COMLCountry.MSG_COUNTRIES_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            for ct in country_list:
                country[ct.get_key()] = ct

            self.get_logger().info(COMLCountry.MSG_COUNTRIES_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLCountry.MSG_COUNTRIES_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return country

    def load_country(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()

            country: COCountry = dbsession.query(COCountry).get(identifier)

            if country is None:
                message = COMLCountry.MSG_COUNTRY_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLCountry.MSG_COUNTRY_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    country.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLCountry.MSG_COUNTRY_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return country

    def add_country(self, country: COCountry):
        dbsession = None
        try:

            if not isinstance(country, COCountry):
                message = COMLCountry.MSG_COUNTRY_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    country.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            dbsession.add(country)
            dbsession.commit()

            self.get_logger().info(
                COMLCountry.MSG_COUNTRY_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    country.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLCountry.MSG_COUNTRY_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                country.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_country(self, country: COCountry):
        dbsession = None
        try:

            if not isinstance(country, COCountry):
                message = COMLCountry.MSG_COUNTRY_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    country.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_ountry: COCountry = dbsession.query(COCountry).get(country.identifier)
            db_ountry.identifier = country.identifier
            db_ountry.name = country.name

            dbsession.commit()

            self.get_logger().info(
                COMLCountry.MSG_COUNTRY_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    country.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLCountry.MSG_COUNTRY_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                country.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_country(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_ountry: COCountry = dbsession.query(COCountry).get(int)
            dbsession.delete(db_ountry)
            dbsession.commit()

            self.get_logger().info(
                COMLCountry.MSG_COUNTRY_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLCountry.MSG_COUNTRY_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
