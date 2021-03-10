from datetime import datetime

from sqlalchemy.ext.declarative import declarative_base

ORMBase = declarative_base()


class COBase(object):
    __create_id = 0
    __create_ts = datetime.now()
    __update_id = 0
    __update_ts = datetime.now()

    def __init__(self):
        self.touched = False
        self.mark_for_deletion = False
        self.new = False

    def get_key(self):
        pass

    @property
    def touched(self):
        return self.__touched

    @touched.setter
    def touched(self, touched):
        self.__touched = touched

    @property
    def new(self):
        return self.__new

    @new.setter
    def new(self, new):
        self.__new = new

    @property
    def mark_for_deletion(self):
        return self.__mark_for_deletion

    @mark_for_deletion.setter
    def mark_for_deletion(self, mark_for_deletion):
        self.__mark_for_deletion = mark_for_deletion

    @property
    def create_id(self):
        return self.__create_id

    @create_id.setter
    def create_id(self, create_id):
        self.__create_id = create_id

    @property
    def create_ts(self):
        return self.__create_ts

    @create_ts.setter
    def create_ts(self, create_ts):
        if create_ts is None:
            self.__create_ts = datetime.now()
        else:
            self.__create_ts = create_ts

    @property
    def update_id(self):
        return self.__update_id

    @update_id.setter
    def update_id(self, update_id):
        self.__update_id = update_id

    @property
    def update_ts(self):
        return self.__update_ts

    @update_ts.setter
    def update_ts(self, update_ts):
        if update_ts is None:
            self.__update_ts = datetime.now()
        else:
            self.__update_ts = update_ts


class COEmpty(COBase):

    @property
    def serialize(self):
        return None
