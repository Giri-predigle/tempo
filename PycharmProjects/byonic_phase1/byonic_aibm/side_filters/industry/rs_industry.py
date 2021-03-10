from flask import request, jsonify

from byonic_aibm.side_filters.industry.co_industry import COIndustry
from byonic_aibm.side_filters.industry.co_industry import COMLIndustry
from byonic_aibm.side_filters.industry.bl_industry import BLIndustry
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_industry: BLIndustry = BLIndustry()


def get_industries():
    try:
        industries = bl_industry.get_industries()
        response = jsonify(
            industrys=[industries[industry_key].serialize for industry_key in industries])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLIndustry.MSG_INDUSTRIES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_industries_by_name():
    try:
        industries = bl_industry.get_industries()
        industries = [industries[industry_key].serialize_by_name for industry_key in industries]
        industry = []
        for i in industries:
            industry.append(list(i.values()))
        industry = [item for t in industry for item in t]
        industries = industry
        response = {'industries': industries}
    except CFException as cfe:
            raise cfe
    except Exception as e:
        message = COMLIndustry.MSG_INDUSTRIES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_industry(identifier):
    try:
        industry = bl_industry.get_industry(identifier=identifier)
        response = jsonify(industry=industry.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLIndustry.MSG_INDUSTRY_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_industry():
    try:
        jsondata = get_request_json()
        if not jsondata or not COIndustry.validate_json(jsondata):
            message = COMLIndustry.MSG_INDUSTRY_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message)
        industry = COIndustry.deserialize(jsondata)
        industry.identifier = None
        industry = bl_industry.create_industry(industry)
        response = jsonify(industry=industry.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLIndustry.MSG_INDUSTRY_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_industry(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COIndustry.validate_json(jsondata):
            message = COMLIndustry.MSG_INDUSTRY_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message)
        industry = COIndustry.deserialize(jsondata)
        industry.identifier = identifier
        industry = bl_industry.update_industry(industry)
        response = jsonify(Industry=industry.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLIndustry.MSG_INDUSTRY_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_industry(identifier):
    try:
        bl_industry.delete_industry(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLIndustry.MSG_INDUSTRY_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLIndustry.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
