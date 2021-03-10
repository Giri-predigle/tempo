from flask import request, jsonify

from byonic_aibm.side_filters.country.co_country import COMLCountry, COCountry
from byonic_aibm.side_filters.country.bl_country import BLCountry
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_country: BLCountry = BLCountry()


def get_countries():
    try:
        countries = bl_country.get_countries()
        response = jsonify(
            countries=[countries[country_key].serialize for country_key in countries])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLCountry.MSG_COUNTRIES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_countries_by_name():
    try:
        countries = bl_country.get_countries()
        countries = [countries[country_key].serialize_by_name for country_key in countries]
        country = []
        for i in countries:
            country.append(list(i.values()))
        country = [item for t in country for item in t]
        countries = country
        response = {'countries': countries}
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLCountry.MSG_COUNTRIES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_country(identifier):
    try:
        countries = bl_country.get_country(identifier=identifier)
        response = jsonify(
            countries=countries.serialize
        )
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLCountry.MSG_COUNTRY_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_country():
    try:
        jsondata = get_request_json()
        if not jsondata or not COCountry.validate_json(jsondata):
            message = COMLCountry.MSG_COUNTRY_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        country = COCountry.deserialize(jsondata)
        country.identifier = None
        country = bl_country.create_country(country)
        response = jsonify(country=country.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLCountry.MSG_COUNTRY_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_country(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COCountry.validate_json(jsondata):
            message = COMLCountry.MSG_COUNTRY_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        country = COCountry.deserialize(jsondata)
        country.identifier = identifier
        country = bl_country.update_country(country)
        response = jsonify(country=country.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLCountry.MSG_COUNTRY_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_country(identifier):
    try:
        bl_country.delete_country(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLCountry.MSG_COUNTRY_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLCountry.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
