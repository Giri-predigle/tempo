from byonic_aibm.side_filters.employee_size.co_employee_size import COEmployee, COMLEmployee
from byonic_aibm.side_filters.employee_size.da_employee_size import DAEmployee
from byonic_core.fw_bizlogic import BLBase
from byonic_core.fw_cache import CFCacheManager, CFBaseCache
from byonic_core.fw_exception import CFException
from byonic_core.fw_logging import CFLogger


class BLEmployee(BLBase):
    __logger = None

    def __init__(self):
        super().__init__()
        BLEmployee.__logger = CFLogger().get_logger(self.get_class_name())
        self.cache: CFBaseCache = CFCacheManager().get_cache()
        self.da_employee_size: DAEmployee = DAEmployee()

    @classmethod
    def get_class_name(cls):
        return cls.__name__

    # noinspection PyMethodMayBeStatic
    def get_logger(self):
        return BLEmployee.__logger

    def get_employee_sizes(self):
        employee_sizes = self.da_employee_size.load_all()
        if employee_sizes is not None:
            for employee_size_key in employee_sizes.keys():
                employee_size = employee_sizes[employee_size_key]
                self.cache.put(employee_size_key, employee_size)
        return employee_sizes

    def get_employee_size(self, identifier):
        employee_size_key = COEmployee.create_key(identifier=identifier)
        employee_size = self.cache.get(employee_size_key)
        if employee_size is None:
            employee_size = self.da_employee_size.load_employee_size(identifier)
            self.validate_employee_size(employee_size)
            self.cache.put(employee_size.get_key(), employee_size)
        return employee_size

    def create_employee_size(self, employee_size):
        self.validate_employee_size(employee_size)
        self.da_employee_size.add_employee_size(employee_size)
        if employee_size.identifier is None:
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_INSERT_FAILED
            self.get_logger().error(message)
            raise CFException(ValueError(message), message)
        return employee_size

    def update_employee_size(self, employee_size):
        self.validate_employee_size(employee_size)
        self.da_employee_size.update_employee_size(employee_size)
        self.cache.remove(employee_size.get_key())
        return employee_size

    def delete_employee_size(self, identifier):
        self.da_employee_size.delete_employee_size(identifier)
        employee_size_key = COEmployee.create_key(identifier=identifier)
        self.cache.remove(employee_size_key)
        return identifier

    def save_employee_sizes(self):
        for employee_size in self.get_employee_sizes():
            self.save_employee_size(employee_size)

    def save_employee_size(self, employee_size):
        self.validate_employee_size(employee_size)
        if employee_size.mark_for_deletion:
            self.delete_employee_size(employee_size.identifier)
        elif employee_size.touched:
            if employee_size.identifier is None:
                self.create_employee_size(employee_size)
            else:
                self.update_employee_size(employee_size)
            employee_size.touched = False

    def validate_employee_size(self, employee_size):
        if not isinstance(employee_size, COEmployee):
            message = COMLEmployee.MSG_EMPLOYEE_VALUE_NOT_VALID
            self.get_logger().error(message)
            raise CFException(ValueError(message), message, status_code=CFException.HTTP_400_BAD_REQUEST)
