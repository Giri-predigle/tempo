from byonic_aibm.side_filters.job_levels.co_job_level import COJobLevel, COMLJobLevel

from byonic_core.fw_dataaccess import DABase
from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COStringLiteral


class DAJobLevel(DABase):
    __logger = None

    def __init__(self):
        super().__init__()
        DAJobLevel.__logger = CFLogger().get_logger(self.get_class_name())

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return DAJobLevel.__logger

    def load_all(self):
        dbsession = None
        job_level = dict()
        try:
            dbsession = DBConnection().getsession()
            job_level_list = dbsession.query(COJobLevel).all()

            if job_level_list is None:
                message = COMLJobLevel.MSG_JOB_LEVELS_NOT_FOUND
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            for ct in job_level_list:
                job_level[ct.get_key()] = ct

            self.get_logger().info(COMLJobLevel.MSG_JOB_LEVELS_LOAD_SUCCESS)

        except CFException:
            raise
        except Exception:
            message = COMLJobLevel.MSG_JOB_LEVELS_LOAD_FAILED
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return job_level

    def load_job_level(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            job_level: COJobLevel = dbsession.query(COJobLevel).get(identifier)

            if job_level is None:
                message = COMLJobLevel.MSG_JOB_LEVEL_NOT_FOUND + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_404_NOT_FOUND)

            self.get_logger().info(
                COMLJobLevel.MSG_JOB_LEVEL_LOAD_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    job_level.identifier))

        except CFException:
            raise
        except Exception:
            message = COMLJobLevel.MSG_JOB_LEVEL_LOAD_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
        return job_level

    def add_job_level(self, job_level: COJobLevel):
        dbsession = None
        try:

            if not isinstance(job_level, COJobLevel):
                message = COMLJobLevel.MSG_JOB_LEVEL_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    job_level.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            dbsession.add(job_level)
            dbsession.commit()

            self.get_logger().info(
                COMLJobLevel.MSG_JOB_LEVEL_INSERT_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    job_level.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLJobLevel.MSG_JOB_LEVEL_INSERT_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                job_level.identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def update_job_level(self, job_level: COJobLevel):
        dbsession = None
        try:

            if not isinstance(job_level, COJobLevel):
                message = COMLJobLevel.MSG_JOB_LEVEL_NOT_VALID + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    job_level.identifier)
                self.get_logger().error(message)
                raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)

            dbsession = DBConnection().getsession()
            db_job_level: COJobLevel = dbsession.query(COJobLevel).get(job_level.identifier)
            db_job_level.identifier = job_level.identifier
            db_job_level.name = job_level.name

            dbsession.commit()

            self.get_logger().info(
                COMLJobLevel.MSG_JOB_LEVEL_UPDATE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    job_level.identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLJobLevel.MSG_JOB_LEVEL_UPDATE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                job_level.identifier)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()

    def delete_job_level(self, identifier: int):
        dbsession = None
        try:
            dbsession = DBConnection().getsession()
            db_job_level: COJobLevel = dbsession.query(COJobLevel).get(identifier)
            dbsession.delete(db_job_level)
            dbsession.commit()

            self.get_logger().info(
                COMLJobLevel.MSG_JOB_LEVEL_DELETE_SUCCESS + COStringLiteral.STR_SEPARATOR_PIPE + str(
                    identifier))

        except CFException:
            dbsession.rollback()
            raise
        except Exception:
            dbsession.rollback()
            message = COMLJobLevel.MSG_JOB_LEVEL_DELETE_FAILED + COStringLiteral.STR_SEPARATOR_PIPE + str(
                identifier)
            self.get_logger().error(message)
            raise CFException(Exception(message), message)
        finally:
            dbsession.close()
