from datetime import datetime

from byonic_object.co_base import COBase
from byonic_object.co_literal import COEnumLiteral, COStringLiteral, COAppLiteral


class COSchedule(COBase):

    def __init__(self, name, frequency, start_date, end_date, start_time, end_time, identifier=None,
                 create_id=None, create_ts=None, update_id=None, update_ts=None):
        super().__init__()
        self.identifier = identifier
        self.name = name
        self.frequency = frequency
        self.start_date = start_date
        self.end_date = end_date
        self.start_time = start_time
        self.end_time = end_time
        self.create_id = create_id
        self.create_ts = create_ts
        self.update_id = update_id
        self.update_ts = update_ts

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    def get_key(self):
        return COSchedule.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(self.identifier)

    @staticmethod
    def create_key(identifier):
        return COSchedule.get_class_name() + COStringLiteral.STR_SEPARATOR_PIPE + str(identifier)

    @property
    def identifier(self):
        return self.__identifier

    @identifier.setter
    def identifier(self, identifier):
        self.__identifier = identifier

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def frequency(self):
        return self.__frequency

    @frequency.setter
    def frequency(self, frequency):
        self.__frequency = frequency

    @property
    def start_date(self):
        return self.__start_date

    @start_date.setter
    def start_date(self, start_date):
        self.__start_date = start_date

    @property
    def end_date(self):
        return self.__end_date

    @end_date.setter
    def end_date(self, end_date):
        self.__end_date = end_date

    @property
    def start_time(self):
        return self.__start_time

    @start_time.setter
    def start_time(self, start_time):
        self.__start_time = start_time

    @property
    def end_time(self):
        return self.__end_time

    @end_time.setter
    def end_time(self, end_time):
        self.__end_time = end_time

    @property
    def is_daily(self):
        return self.frequency == COEnumLiteral.EN_SCHEDULE_DAILY

    @property
    def is_weekly(self):
        return self.frequency == COEnumLiteral.EN_SCHEDULE_WEEKLY

    @property
    def is_monthly(self):
        return self.frequency == COEnumLiteral.EN_SCHEDULE_MONTHLY

    @property
    def is_yearly(self):
        return self.frequency == COEnumLiteral.EN_SCHEDULE_YEARLY

    @property
    def is_fixed(self):
        return self.frequency == COEnumLiteral.EN_SCHEDULE_FIXED

    def __deepcopy__(self, memo):  # memo is a dict of id's to copies
        id_self = id(self)  # memoization avoids unnecesary recursion
        _copy = memo.get(id_self)
        if _copy is None:
            _copy = type(self)(
                self.name,
                self.frequency,
                self.start_date,
                self.end_date,
                self.start_time,
                self.end_time,
                None,
                COAppLiteral.APP_SYS_USER_ID,
                None,
                COAppLiteral.APP_SYS_USER_ID,
                None)
            _copy.__type = self.__type
            memo[id_self] = _copy
        return _copy

    @property
    def serialize(self):
        return {'identifer': self.identifier,
                'name': self.name,
                'frequency': self.frequency,
                'start_date': str(self.start_date),
                'end_date': str(self.end_date),
                'start_time': str(self.start_time),
                'end_time': str(self.end_time),
                'create_id': self.create_id,
                'create_ts': str(self.create_ts),
                'update_id': self.update_id,
                'update_ts': str(self.update_ts)}

    @classmethod
    def deserialize(cls, jsondata):
        identifier = None
        create_id = COAppLiteral.APP_SYS_USER_ID
        create_ts = datetime.now()
        update_id = COAppLiteral.APP_SYS_USER_ID
        update_ts = datetime.now()
        name = jsondata['name']
        frequency = jsondata['frequency']
        start_date = jsondata['start_date']
        end_date = jsondata['end_date']
        start_time = jsondata['start_time']
        end_time = jsondata['end_time']
        if 'identifier' in jsondata:
            identifier = jsondata['identifier']
        if 'create_id' in jsondata:
            create_id = jsondata['create_id']
        if 'create_ts' in jsondata:
            create_ts = jsondata['create_ts']
        if 'update_id' in jsondata:
            update_id = jsondata['update_id']
        if 'update_ts' in jsondata:
            update_ts = jsondata['update_ts']
        schedule = COSchedule(name=name, frequency=frequency, start_date=start_date,
                              end_date=end_date, start_time=start_time,
                              end_time=end_time, identifier=identifier,
                              create_id=create_id, create_ts=create_ts,
                              update_id=update_id, update_ts=update_ts)
        return schedule

    @classmethod
    def validate_json(cls, jsondata):
        if 'name' not in jsondata:
            return False
        if 'frequency' not in jsondata:
            return False
        if 'start_date' not in jsondata:
            return False
        if 'end_date' not in jsondata:
            return False
        if 'start_time' not in jsondata:
            return False
        if 'end_time' not in jsondata:
            return False
        if 'create_id' not in jsondata:
            return False
        if 'update_id' not in jsondata:
            return False
        return True
