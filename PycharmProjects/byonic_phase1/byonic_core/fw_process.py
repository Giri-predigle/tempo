
import functools
import threading
import time
from queue import Queue

import schedule

from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger
from byonic_object.co_literal import COMessageLiteral, COKeyLiteral
from byonic_object.co_schedule import COSchedule


class SCBase(object):

    def schedule(self):
        pass


class PSBase(object):

    def run(self):
        pass


class FWQueue(object):
    __queue = None
    __logger = None

    def __init__(self):
        FWQueue.__logger = CFLogger().get_logger(self.get_class_name())
        if FWQueue.__queue is None:
            FWQueue.__queue = Queue()

    def get_class_name(self):
        return self.__class__.__name__

    @staticmethod
    def logger():
        return FWQueue.__logger

    @staticmethod
    def put_item(item):
        FWQueue.__queue.put(item)

    @staticmethod
    def get_item():
        return FWQueue.__queue.get()

    @staticmethod
    def done_item():
        FWQueue.__queue.task_done()

    @staticmethod
    def depth():
        return FWQueue.__queue.qsize()


class FWScheduler(object):
    __jobqueue = None
    __logger = None

    def __init__(self):
        FWScheduler.__logger = CFLogger().get_logger(self.get_class_name())
        self.jobqueue()

    def get_class_name(self):
        return self.__class__.__name__

    @staticmethod
    def logger():
        return FWScheduler.__logger

    @staticmethod
    def jobqueue():
        if FWScheduler.__jobqueue is None:
            FWScheduler.__jobqueue = FWQueue()
        return FWScheduler.__jobqueue

    @staticmethod
    def schedule(process, schedule_info):
        # This decorator can be applied to
        def with_logging(func):
            @functools.wraps(func)
            def wrapper(*args, **kwargs):
                FWScheduler.logger().info('LOG: Running job "%s"' % func.__name__)
                result = func(*args, **kwargs)
                FWScheduler.logger().info('LOG: Job "%s" completed' % func.__name__)
                return result
            return wrapper

        @with_logging
        def run_recurring():
            process.run()

        @with_logging
        def run_once():
            process.run()
            return schedule.CancelJob

        # validate process and schedule
        if not isinstance(process, PSBase):
            message = COMessageLiteral.MSG_PROCESS_NOT_VALID
            FWScheduler.logger().error(message)
            raise CFException(ValueError(message), message)
        if not isinstance(schedule_info, COSchedule):
            message = COMessageLiteral.MSG_SCHEDULE_NOT_VALID
            FWScheduler.logger().error(message)
            raise CFException(ValueError(message), message)

        ps_start_time = schedule_info.start_time.strftime('%H:%M')
        queueput = FWScheduler.jobqueue().put_item
        bottag = COKeyLiteral.KEY_OBJECT_REPORT_ALL

        if schedule_info.is_fixed:
            schedule.every().day.at(ps_start_time).do(queueput, run_once).tag(bottag)
        if schedule_info.is_daily:
            schedule.every().day.at(ps_start_time).do(queueput, run_recurring).tag(bottag)

    @staticmethod
    def run_pending():
        schedule.run_pending()

    @staticmethod
    def run_all():
        schedule.run_all()


def worker_main():
    while 1:
        job_func = FWScheduler.jobqueue().get_item()
        job_func()
        FWScheduler.jobqueue().done_item()


worker_thread = threading.Thread(target=worker_main)
worker_thread.start()


def run_continuously(interval=10):
    cease_continuous_run = threading.Event()

    class FWSchedulerThread(threading.Thread):
        @classmethod
        def run(cls):
            while not cease_continuous_run.is_set():
                FWScheduler.run_pending()
                time.sleep(interval)

    continuous_thread = FWSchedulerThread()
    continuous_thread.start()
    return cease_continuous_run


try:
    run_continuously(10)
except (KeyboardInterrupt, SystemExit):
    schedule.clear(COKeyLiteral.KEY_OBJECT_REPORT_ALL)
    print('Jobs Cleared...')
