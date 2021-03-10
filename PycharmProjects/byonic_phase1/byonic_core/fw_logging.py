import logging

from byonic_core.fw_config import CFAppConfig
from byonic_object.co_literal import COMessageLiteral, COAppLiteral


class CFLogger:
    loggers = None

    def __init__(self):
        if CFLogger.loggers is None:
            self.logger = None
            CFLogger.loggers = dict()
            self.__init_logger(CFAppConfig().get_app_name())
            # logging.basicConfig()
            self.__init_logger('sqlalchemy')

    # noinspection PyBroadException
    def __init_logger(self, name):
        try:
            # create logger with name
            self.logger = logging.getLogger(name)
            self.logger.setLevel(CFAppConfig().get_log_level())
            # create file handler which logs even debug messages
            fh = logging.FileHandler(CFAppConfig().get_log_filename())
            fh.setLevel(CFAppConfig().get_log_level())
            # create console handler with a higher log level
            ch = logging.StreamHandler()
            ch.setLevel(CFAppConfig().get_log_level())
            # create formatter and add it to the handlers
            formatter = logging.Formatter(COAppLiteral.APP_CONFIG_LOG_FORMAT)
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            # add the handlers to the logger
            self.logger.addHandler(fh)
            self.logger.addHandler(ch)
            CFLogger.loggers[name] = self.logger
        except:
            message = COMessageLiteral.MSG_LOGGER_INIT_FAILED
            print(Exception(message), message)

    def get_logger(self, name):
        if name not in CFLogger.loggers.keys():
            self.__init_logger(name)
        return CFLogger.loggers[name]
