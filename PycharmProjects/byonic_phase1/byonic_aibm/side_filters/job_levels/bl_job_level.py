from byonic_aibm.side_filters.job_levels.co_job_level import COJobLevel
from byonic_aibm.side_filters.job_levels.co_job_level import COMLJobLevel

from byonic_aibm.side_filters.job_levels.da_job_level import DAJobLevel
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLJobLevel(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLJobLevel.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_job_level: DAJobLevel = DAJobLevel()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLJobLevel.__logger

    def get_job_levels(self):
        job_levels = self.da_job_level.load_all()
        if job_levels is not None:
            for job_level_key in job_levels.keys():
                job_level = job_levels[job_level_key]
                self.cache.put(job_level_key, job_level)
        return job_levels

    def get_job_level(self, identifier):
        job_level_key = COJobLevel.create_key(identifier=identifier)
        job_level = self.cache.get(job_level_key)
        if job_level is None:
            job_level = self.da_job_level.load_job_level(identifier)
            self.validate_job_level(job_level)
            self.cache.put(job_level.get_key(), job_level)
        return job_level

    def create_job_level(self, job_level):
        self.validate_job_level(job_level)
        self.da_job_level.add_job_level(job_level)
        if job_level.identifier is None:
            message = COMLJobLevel.MSG_JOB_LEVEL_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return job_level

    def update_job_level(self, job_level):
        self.validate_job_level(job_level)
        self.da_job_level.update_job_level(job_level)
        self.cache.remove(job_level.get_key())
        return job_level

    def delete_job_level(self, identifier):
        self.da_job_level.delete_job_level(identifier)
        job_level_key = COJobLevel.create_key(identifier=identifier)
        self.cache.remove(job_level_key)
        return identifier

    def save_job_levels(self):
        for job_level in self.get_job_levels():
            self.save_job_level(job_level)

    def save_job_level(self, job_level):
        self.validate_job_level(job_level)
        if job_level.mark_for_deletion:
            self.delete_job_level(job_level.identifier)
        elif job_level.touched:
            if job_level.identifier is None:
                self.create_job_level(job_level)
            else:
                self.update_job_level(job_level)
            job_level.touched = False

    def validate_job_level(self, job_level):
        if not isinstance(job_level, COJobLevel):
            message = COMLJobLevel.MSG_JOB_LEVEL_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
