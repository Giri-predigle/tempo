
from byonic_core.fw_config import CFAppConfig


class CFBaseCache:

    def __init__(self):
        self.cache = None
        self.enabled = False

    @property
    def cache(self):
        return self.__cache

    @cache.setter
    def cache(self, cache):
        self.__cache = cache

    @property
    def enabled(self):
        return self.__enabled

    @enabled.setter
    def enabled(self, enabled):
        self.__enabled = enabled

    def get(self, key):
        pass

    def put(self, key, value):
        pass

    def remove(self, key):
        pass

    def exists(self, key):
        pass


class CFDictCache(CFBaseCache):

    def __init__(self, enabled):
        super().__init__()
        self.cache = dict()
        self.enabled = enabled

    def get(self, key):
        if self.enabled:
            if self.exists(key):
                return self.cache[key]
        return None

    def put(self, key, value):
        if self.enabled:
            self.cache[key] = value

    def exists(self, key):
        if self.enabled:
            if key in self.cache.keys():
                return True
        return False

    def remove(self, key):
        if self.enabled:
            if self.exists(key):
                self.cache.pop(key)


class CFCacheManager:

    __cache = None
    __enabled = False

    def __init__(self):
        super().__init__()
        CFCacheManager.__enabled = CFAppConfig().is_cache_enabled()

    @staticmethod
    def get_cache():
        if CFCacheManager.__cache is None:
            CFCacheManager.__cache = CFDictCache(CFCacheManager.__enabled)
        return CFCacheManager.__cache
