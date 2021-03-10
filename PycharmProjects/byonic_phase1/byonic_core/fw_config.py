#!/usr/bin/python
from configparser import ConfigParser

from byonic_core.fw_exception import CFException
from byonic_object.co_framework import COAppProperties
from byonic_object.co_literal import COAppLiteral, COMessageLiteral, COPropLiteral


class CFAppConfig:
    properties = None

    def __init__(self):
        if CFAppConfig.properties is None:
            CFAppConfig.properties = COAppProperties()
            CFAppConfig.__load_properties(COAppLiteral.APP_CONFIG_FILE_LOCATION)

    @staticmethod
    def __load_properties(filename):
        try:
            # create a parser
            parser = ConfigParser()
            # read config file
            parser.read(filename)

            # read sections
            props_app = CFAppConfig.__load_section(parser, COAppLiteral.APP_CONFIG_SECTION_APP)
            props_entity_db = CFAppConfig.__load_section(parser, COAppLiteral.APP_CONFIG_SECTION_ENTITY_DB)
            props_entity_api = CFAppConfig.__load_section(parser, COAppLiteral.APP_CONFIG_SECTION_ENTITY_API)
            props_shared_log = CFAppConfig.__load_section(parser, COAppLiteral.APP_CONFIG_SECTION_SHARED_LOG)
            props_shared_cache = CFAppConfig.__load_section(parser, COAppLiteral.APP_CONFIG_SECTION_SHARED_CACHE)
            props_pool = CFAppConfig.__load_section(parser, COAppLiteral.APP_CONFIG_SECTION_POOLING)

            # initialize properties
            CFAppConfig.properties.application = props_app
            CFAppConfig.properties.entity_db = props_entity_db
            CFAppConfig.properties.entity_api = props_entity_api
            CFAppConfig.properties.shared_logger = props_shared_log
            CFAppConfig.properties.shared_cache = props_shared_cache
            CFAppConfig.properties.shared_pool = props_pool

        except Exception:
            message = COMessageLiteral.MSG_CONFIG_LOAD_FAILED
            raise CFException(Exception(message), message)

    @staticmethod
    def __load_section(parser, section):
        section_props = {}
        if parser.has_section(section):
            params = parser.items(section)
            for param in params:
                section_props[param[0]] = param[1]
        else:
            message = COMessageLiteral.MSG_CONFIG_SECTION_LOAD_FAILED
            raise CFException(ValueError(message), message)
        return section_props

    def get_app_name(self):
        return self.properties.application[COPropLiteral.PROP_APP_NAME]

    def get_app_version(self):
        return self.properties.application[COPropLiteral.PROP_APP_VERSION]

    def get_app_instance(self):
        return self.properties.application[COPropLiteral.PROP_APP_INSTANCE]

    def get_app_sysuser(self):
        return self.properties.application[COPropLiteral.PROP_APP_SYS_USER]

    def get_entity_db_name(self):
        return self.properties.entity_db[COPropLiteral.PROP_ENTITY_DB_NAME]

    def get_entity_db_alias(self):
        return self.properties.entity_db[COPropLiteral.PROP_ENTITY_DB_ALIAS]

    def get_entity_db_host(self):
        return self.properties.entity_db[COPropLiteral.PROP_ENTITY_DB_HOST]

    def get_entity_db_port(self):
        return self.properties.entity_db[COPropLiteral.PROP_ENTITY_DB_PORT]

    def get_entity_db_user(self):
        return self.properties.entity_db[COPropLiteral.PROP_ENTITY_DB_USERNAME]

    def get_entity_db_password(self):
        return self.properties.entity_db[COPropLiteral.PROP_ENTITY_DB_PASSWORD]

    def get_aibm_api_name(self):
        return self.properties.entity_api[COPropLiteral.PROP_ENTITY_API_NAME]

    def get_aibm_api_port(self):
        return self.properties.entity_api[COPropLiteral.PROP_ENTITY_API_PORT]

    def get_log_filename(self):
        return self.properties.shared_logger[COPropLiteral.PROP_SHARED_LOG_FILENAME]

    def get_log_level(self):
        return self.properties.shared_logger[COPropLiteral.PROP_SHARED_LOG_LEVEL]

    def get_page_rows_default(self):
        return self.properties.shared_logger[COPropLiteral.PROP_SHARED_PAGE_ROWS_DEFAULT]

    def get_page_count_default(self):
        return self.properties.shared_logger[COPropLiteral.PROP_SHARED_PAGE_COUNT_DEFAULT]

    def is_cache_enabled(self):
        if self.properties.shared_cache[COPropLiteral.PROP_SHARED_CACHE_ENABLED] == 'True':
            return True
        return False

    def get_pool_size(self):
        return self.properties.shared_pool[COPropLiteral.PROP_POOL_SIZE]


if __name__ == '__main__':
    print(CFAppConfig().properties.application)
    print(CFAppConfig().properties.entity_db)
    print(CFAppConfig().properties.entity_api)
    print(CFAppConfig().properties.shared_logger)
    print(CFAppConfig().properties.shared_cache)
