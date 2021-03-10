from flask import request, jsonify

from byonic_aibm.side_filters.revenue_range.co_revenue_range import CORevenue
from byonic_aibm.side_filters.revenue_range.co_revenue_range import COMLRevenue
from byonic_aibm.side_filters.revenue_range.bl_revenue_range import BLRevenue
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_revenue_range: BLRevenue = BLRevenue()


def get_revenue_range_values():
    try:
        revenue_range_values = bl_revenue_range.get_revenue_ranges()
        response = jsonify(
            revenue_range_values=[revenue_range_values[revenue_range_key].serialize for revenue_range_key in
                                  revenue_range_values])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLRevenue.MSG_REVENUE_RANGES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_revenue_range_values_by_range():
    try:
        revenue_range_values = bl_revenue_range.get_revenue_ranges()
        revenue_range_values = [revenue_range_values[country_key].serialize_by_range for country_key in
                                revenue_range_values]
        value = []
        for i in revenue_range_values:
            value.append(list(i.values()))
        value = [item for t in value for item in t]
        revenue_range_values = value
        response = {"revenues": revenue_range_values}
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLRevenue.MSG_REVENUE_RANGES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_revenue_range(identifier):
    try:
        revenue_range = bl_revenue_range.get_revenue_range(identifier=identifier)
        response = jsonify(revenue_range=revenue_range.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLRevenue.MSG_REVENUE_RANGE_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_revenue_range():
    try:
        jsondata = get_request_json()
        if not jsondata or not CORevenue.validate_json(jsondata):
            message = COMLRevenue.MSG_REVENUE_RANGE_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        revenue_range = CORevenue.deserialize(jsondata)
        revenue_range.identifier = None
        revenue_range = bl_revenue_range.create_revenue_range(revenue_range)
        response = jsonify(revenue_range=revenue_range.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLRevenue.MSG_REVENUE_RANGE_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_revenue_range(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not CORevenue.validate_json(jsondata):
            message = COMLRevenue.MSG_REVENUE_RANGE_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        revenue_range = CORevenue.deserialize(jsondata)
        revenue_range.identifier = identifier
        revenue_range = bl_revenue_range.update_revenue_range(revenue_range)
        response = jsonify(revenue_range=revenue_range.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLRevenue.MSG_REVENUE_RANGE_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_revenue_range(identifier):
    try:
        bl_revenue_range.delete_revenue_range(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLRevenue.MSG_REVENUE_RANGE_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLRevenue.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
