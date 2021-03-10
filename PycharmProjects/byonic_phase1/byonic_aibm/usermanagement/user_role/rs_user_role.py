from flask import request, jsonify

from byonic_aibm.usermanagement.user_role.bl_user_role import BLUserRole
from byonic_aibm.usermanagement.user_role.co_user_role import COUserRole
from byonic_aibm.usermanagement.user_role.co_user_role import COMLUserRole
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_user_role: BLUserRole = BLUserRole()


def get_user_roles():
    try:
        per_page = request.args.get('per_page', None)
        users = bl_user_role.get_user_roles(per_page)
        response = jsonify(
            users=[users[user_key].serialize for user_key in users])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLUserRole.MSG_USER_ROLES_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_user_role(identifier):
    try:
        user = bl_user_role.get_user_role(identifier=identifier)
        response = jsonify(user=user.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLUserRole.MSG_USER_ROLE_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_user_role():
    try:
        jsondata = get_request_json()
        if not jsondata or not COUserRole.validate_json(jsondata):
            message = COMLUserRole.MSG_USER_ROLE_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        user_role = COUserRole.deserialize(jsondata)
        # user_role.identifier = None
        user_role = bl_user_role.create_user_role(user_role)
        response = jsonify(user_role=user_role.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLUserRole.MSG_USER_ROLE_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_user_role(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COUserRole.validate_json(jsondata):
            message = COMLUserRole.MSG_USER_ROLE_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        user_role = COUserRole.deserialize(jsondata)
        user_role.identifier = identifier
        user_role = bl_user_role.update_user_role(user_role)
        response = jsonify(user_role=user_role.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLUserRole.MSG_USER_ROLE_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_user_role(identifier):
    try:
        bl_user_role.delete_user_role(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLUserRole.MSG_USER_ROLE_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLUserRole.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
