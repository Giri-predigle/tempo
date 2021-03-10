from byonic_aibm.side_filters.intent_topics.co_intent_topics import COTopic
from byonic_aibm.side_filters.intent_topics.co_intent_topics import COMLTopic

from byonic_aibm.side_filters.intent_topics.da_intent_topics import DATopic
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLTopic(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLTopic.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_intent_topics: DATopic = DATopic()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLTopic.__logger

    def get_topics(self):
        intent_topic = self.da_intent_topics.load_all()
        if intent_topic is not None:
            for intent_topics_key in intent_topic.keys():
                intent_topics = intent_topic[intent_topics_key]
                self.cache.put(intent_topics_key, intent_topics)
        return intent_topic

    def get_intent_topics(self, identifier):
        intent_topics_key = COTopic.create_key(identifier=identifier)
        intent_topics = self.cache.get(intent_topics_key)
        if intent_topics is None:
            intent_topics = self.da_intent_topics.load_intent_topics(identifier)
            self.validate_intent_topics(intent_topics)
            self.cache.put(intent_topics.get_key(), intent_topics)
        return intent_topics

    def create_intent_topics(self, intent_topics):
        self.validate_intent_topics(intent_topics)
        self.da_intent_topics.add_intent_topics(intent_topics)
        if intent_topics.identifier is None:
            message = COMLTopic.MSG_TOPIC_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return intent_topics

    def update_intent_topics(self, intent_topics):
        self.validate_intent_topics(intent_topics)
        self.da_intent_topics.update_intent_topics(intent_topics)
        self.cache.remove(intent_topics.get_key())
        return intent_topics

    def delete_intent_topics(self, identifier):
        self.da_intent_topics.delete_intent_topics(identifier)
        intent_topics_key = COTopic.create_key(identifier=identifier)
        self.cache.remove(intent_topics_key)
        return identifier

    def save_industries(self):
        for intent_topics in self.get_industries():
            self.save_intent_topics(intent_topics)

    def save_intent_topics(self, intent_topics):
        self.validate_intent_topics(intent_topics)
        if intent_topics.mark_for_deletion:
            self.delete_intent_topics(intent_topics.identifier)
        elif intent_topics.touched:
            if intent_topics.identifier is None:
                self.create_intent_topics(intent_topics)
            else:
                self.update_intent_topics(intent_topics)
            intent_topics.touched = False

    def validate_intent_topics(self, intent_topics):
        if not isinstance(intent_topics, COTopic):
            message = COMLTopic.MSG_TOPIC_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
