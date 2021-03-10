from byonic_aibm.side_filters.intent_topics.co_intent_topics import COTopic
from byonic_aibm.side_filters.intent_topics.co_intent_topics import COMLTopic

from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral


class DATopic(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DATopic.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DATopic.__logger

    def load_all(self):
        dbsession = None
        intent_topics_dict = dict()
        try:
            dbsession = DBConnection().getsession()

            intent_topics_list = dbsession.query(COTopic).all()

            if intent_topics_list is None:
                message = COMLTopic.MSG_TOPICS_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            try:
                for ct in intent_topics_list:
                    for cts in ct:
                        intent_topics_dict[cts.get_key()] = cts
            except TypeError:
                for ct in intent_topics_list:
                    intent_topics_dict[ct.get_key()] = ct

            self.get_logger().info(COMLTopic.MSG_TOPICS_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLTopic.MSG_TOPICS_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return intent_topics_dict

    def load_intent_topics(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            intent_topics: COTopic = dbsession.query(COTopic).get(identifier)

            if intent_topics is None:
                message = COMLTopic.MSG_TOPIC_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLTopic.MSG_TOPIC_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    intent_topics.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLTopic.MSG_TOPIC_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return intent_topics

    def add_intent_topics(self, intent_topics: COTopic):
        dbsession = None
        try:

            if not isinstance(intent_topics, COTopic):
                message = COMLTopic.MSG_TOPIC_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    intent_topics.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            dbsession.add(intent_topics)
            dbsession.commit()

            self.get_logger().info(
                COMLTopic.MSG_TOPIC_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    intent_topics.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLTopic.MSG_TOPIC_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                intent_topics.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_intent_topics(self, intent_topics: COTopic):
        dbsession = None
        try:

            if not isinstance(intent_topics, COTopic):
                message = COMLTopic.MSG_TOPIC_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    intent_topics.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_intent_topics: COTopic = dbsession.query(COTopic).get(intent_topics.identifier)
            db_intent_topics.identifier = intent_topics.identifier
            db_intent_topics.name = intent_topics.name

            dbsession.commit()

            self.get_logger().info(
                COMLTopic.MSG_TOPIC_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    intent_topics.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLTopic.MSG_TOPIC_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                intent_topics.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_intent_topics(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_intent_topics: COTopic = dbsession.query(COTopic).get(identifier)
            dbsession.delete(db_intent_topics)
            dbsession.commit()

            self.get_logger().info(
                COMLTopic.MSG_TOPIC_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLTopic.MSG_TOPIC_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
