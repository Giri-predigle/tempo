from byonic_object.co_base import COBase


class COAppProperties(COBase):

    def __init__(self):
        super().__init__()
        self.application = dict()
        self.logger = dict()
        self.database = dict()

    @property
    def application(self):
        return self.__application

    @application.setter
    def application(self, application):
        self.__application = application

    @property
    def database(self):
        return self.__database

    @database.setter
    def database(self, database):
        self.__database = database

    @property
    def logger(self):
        return self.__logger

    @logger.setter
    def logger(self, logger):
        self.__logger = logger
