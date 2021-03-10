from flask import request, jsonify

from byonic_aibm.side_filters.job_levels.co_job_level import COJobLevel
from byonic_aibm.side_filters.job_levels.co_job_level import COMLJobLevel
from byonic_aibm.side_filters.job_levels.bl_job_level import BLJobLevel
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_job_level: BLJobLevel = BLJobLevel()


def get_job_levels():
    try:
        job_levels = bl_job_level.get_job_levels()
        response = jsonify(
            job_levels=[job_levels[job_level_key].serialize for job_level_key in job_levels])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLJobLevel.MSG_JOB_LEVELS_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_job_levels_by_name():
    try:
        job_levels = bl_job_level.get_job_levels()
        job_levels = [job_levels[country_key].serialize_by_name for country_key in job_levels]
        level = []
        for i in job_levels:
            level.append(list(i.values()))
        level = [item for t in level for item in t]
        job_levels = level
        response = {"job_levels": job_levels}
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLJobLevel.MSG_JOB_LEVELS_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_job_level(identifier):
    try:
        job_level = bl_job_level.get_job_level(identifier=identifier)
        response = jsonify(job_level=job_level.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLJobLevel.MSG_JOB_LEVEL_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_job_level():
    try:
        jsondata = get_request_json()
        if not jsondata or not COJobLevel.validate_json(jsondata):
            message = COMLJobLevel.MSG_JOB_LEVEL_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message)
        job_level = COJobLevel.deserialize(jsondata)
        job_level.identifier = None
        job_level = bl_job_level.create_job_level(job_level)
        response = jsonify(job_level=job_level.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLJobLevel.MSG_JOB_LEVEL_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_job_level(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COJobLevel.validate_json(jsondata):
            message = COMLJobLevel.MSG_JOB_LEVEL_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message)
        job_level = COJobLevel.deserialize(jsondata)
        job_level.identifier = identifier
        job_level = bl_job_level.update_job_level(job_level)
        response = jsonify(Industry=job_level.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLJobLevel.MSG_JOB_LEVEL_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_job_level(identifier):
    try:
        bl_job_level.delete_job_level(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLJobLevel.MSG_JOB_LEVEL_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLJobLevel.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
