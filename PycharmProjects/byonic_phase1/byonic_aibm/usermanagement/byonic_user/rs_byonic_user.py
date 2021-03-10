from flask import request, jsonify
import jwt

from byonic_core.fw_exception import CFException
from byonic_aibm.usermanagement.byonic_user.bl_byonic_user import BLByonicUser
from byonic_aibm.usermanagement.byonic_user.co_byonic_user import COByonicUser
from byonic_aibm.usermanagement.byonic_user.co_byonic_user import COMLByonicUser

from byonic_core.fw_logging import CFLogger

bl_byonic_user: BLByonicUser = BLByonicUser()
logger = CFLogger().get_logger(__name__)


def get_byonic_users(identifier):
    try:
        byonic_users = bl_byonic_user.get_byonic_users(identifier)
        print("Aravind", byonic_users)
        b = [byonic_users[byonic_user_key].serialize for byonic_user_key in byonic_users]
        print("Aravind", b)
        response = jsonify(
            [byonic_users[byonic_user_key].serialize for byonic_user_key in byonic_users])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLByonicUser.MSG_BYONIC_USERS_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_byonic_user(identifier):
    try:
        byonic_user = bl_byonic_user.get_byonic_user(identifier=identifier)
        response = jsonify(byonic_user=byonic_user.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLByonicUser.MSG_BYONIC_USER_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def login_byonic_users():
    try:
        jsondata = get_request_json()
        if not jsondata or not COByonicUser.validate_json(jsondata):
            message = COMLByonicUser.MSG_BYONIC_USER_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

        mail = jsondata['email']
        password = jsondata['password']
        byonic_users = bl_byonic_user.login_byonic_users(mail, password)
        user_db = [byonic_users[byonic_user_key].serialize_login for byonic_user_key in byonic_users]
        print(user_db)
        user_db = jwt.encode({'token': user_db}, "byonicsecret", algorithm="HS256")
        response = jsonify(user_db.decode("utf-8"))
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLByonicUser.MSG_BYONIC_USER_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_byonic_user():
    try:
        jsondata = get_request_json()
        # print(jsondata)
        if not jsondata or not COByonicUser.validate_json(jsondata):
            message = COMLByonicUser.MSG_BYONIC_USER_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        byonic_user = COByonicUser.deserialize(jsondata)
        byonic_user.identifier = None
        byonic_user.role_id = jsondata['role_id']['identifier']
        byonic_user.organisation_id = jsondata['org_id']['identifier']
        byonic_user = bl_byonic_user.create_byonic_user(byonic_user)
        response = jsonify(byonic_user=byonic_user.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLByonicUser.MSG_BYONIC_USER_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_byonic_user(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COByonicUser.validate_json(jsondata):
            message = COMLByonicUser.MSG_BYONIC_USER_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
        byonic_user = COByonicUser.deserialize(jsondata)
        byonic_user.identifier = identifier
        byonic_user = bl_byonic_user.update_byonic_user(byonic_user)
        response = jsonify(byonic_user=byonic_user.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLByonicUser.MSG_BYONIC_USER_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_byonic_user(identifier):
    try:
        bl_byonic_user.delete_byonic_user(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLByonicUser.MSG_BYONIC_USER_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLByonicUser.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
