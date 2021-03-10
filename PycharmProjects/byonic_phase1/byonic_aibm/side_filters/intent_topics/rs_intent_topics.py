from flask import request, jsonify

from byonic_aibm.side_filters.intent_topics.co_intent_topics import COTopic
from byonic_aibm.side_filters.intent_topics.co_intent_topics import COMLTopic
from byonic_aibm.side_filters.intent_topics.bl_intent_topics import BLTopic
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger

logger = CFLogger().get_logger(__name__)
bl_intent_topics: BLTopic = BLTopic()


def get_topics():
    try:
        intent_topics = bl_intent_topics.get_topics()
        response = jsonify(
            intent_topics=[intent_topics[intent_topics_key].serialize for intent_topics_key in intent_topics])
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLTopic.MSG_TOPICS_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_topics_by_name():
    try:
        intent_topics = bl_intent_topics.get_topics()
        intent_topics = [intent_topics[topic_key].serialize_by_name for topic_key in intent_topics]
        topic = []
        for i in intent_topics:
            topic.append(list(i.values()))
        topic = [item for t in topic for item in t]
        intent_topics = topic
        response = {"topics": intent_topics}

    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLTopic.MSG_TOPICS_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_intent_topics(identifier):
    try:
        intent_topics = bl_intent_topics.get_intent_topics(identifier=identifier)
        response = jsonify(intent_topics=intent_topics.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLTopic.MSG_TOPIC_LOAD_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def create_intent_topics():
    try:
        jsondata = get_request_json()
        if not jsondata or not COTopic.validate_json(jsondata):
            message = COMLTopic.MSG_TOPIC_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message)
        intent_topics = COTopic.deserialize(jsondata)
        intent_topics.identifier = None
        intent_topics = bl_intent_topics.create_intent_topics(intent_topics)
        response = jsonify(intent_topics=intent_topics.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLTopic.MSG_TOPIC_INSERT_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def update_intent_topics(identifier):
    try:
        jsondata = get_request_json()
        if not jsondata or not COTopic.validate_json(jsondata):
            message = COMLTopic.MSG_TOPIC_NOT_VALID
            logger.error(message)
            raise CFException(ValueError(message), message)
        intent_topics = COTopic.deserialize(jsondata)
        intent_topics.identifier = identifier
        intent_topics = bl_intent_topics.update_intent_topics(intent_topics)
        response = jsonify(Topic=intent_topics.serialize)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLTopic.MSG_TOPIC_UPDATE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def delete_intent_topics(identifier):
    try:
        bl_intent_topics.delete_intent_topics(identifier)
        response = jsonify(identifier=identifier)
    except CFException as cfe:
        raise cfe
    except Exception as e:
        message = COMLTopic.MSG_TOPIC_DELETE_FAILED
        logger.error(message, e)
        cfe = CFException(SystemError(message), message)
        raise cfe
    return response


def get_request_json():
    try:
        return request.get_json()
    except:
        message = COMLTopic.MSG_REQUEST_JSON_NOT_VALID
        logger.error(message)
        raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
