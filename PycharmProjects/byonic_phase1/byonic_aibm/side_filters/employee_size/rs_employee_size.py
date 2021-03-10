from flask import request, jsonify

from byonic_aibm.side_filters.employee_size.co_employee_size import COEmployee, COMLEmployee
from byonic_aibm.side_filters.employee_size.bl_employee_size import BLEmployee
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_employee: BLEmployee = BLEmployee()


def get_employee_sizes():
    try:
        employee_sizes = bl_employee.get_employee_sizes()
        response = jsonify(
            Employees=[employee_sizes[Employee_key].serialize for Employee_key in employee_sizes])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLEmployee.MSG_EMPLOYEE_VALUES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_employee_sizes_by_value():
    try:
        employee_sizes = bl_employee.get_employee_sizes()
        employee_sizes = [employee_sizes[country_key].serialize_by_value for country_key in employee_sizes]
        value = []
        for i in employee_sizes:
            value.append(list(i.values()))
        value = [item for t in value for item in t]
        employee_sizes = value
        response = {'employee_sizes': employee_sizes}
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLEmployee.MSG_EMPLOYEE_VALUES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_employee_size(identifier):
    try:
        employee_size = bl_employee.get_employee_size(identifier=identifier)
        response = jsonify(employee_size=employee_size.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLEmployee.MSG_EMPLOYEE_VALUE_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_employee_size():
    try:
        jsondata = get_request_json()
        if not jsondata or not COEmployee.validate_json(jsondata):
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        employee_size = COEmployee.deserialize(jsondata)
        employee_size.identifier = None
        employee_size = bl_employee.create_employee_size(employee_size)
        response = jsonify(employee_size=employee_size.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLEmployee.MSG_EMPLOYEE_VALUE_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_employee_size(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COEmployee.validate_json(jsondata):
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        employee_size = COEmployee.deserialize(jsondata)
        employee_size.identifier = identifier
        employee_size = bl_employee.update_employee_size(employee_size)
        response = jsonify(employee_size=employee_size.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLEmployee.MSG_EMPLOYEE_VALUE_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_employee_size(identifier):
    try:
        bl_employee.delete_employee_size(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLEmployee.MSG_EMPLOYEE_VALUE_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLEmployee.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
