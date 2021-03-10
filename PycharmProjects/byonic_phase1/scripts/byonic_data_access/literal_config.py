from scripts.DB_config import Db_config

api_config_db = Db_config()

class AppLiteral:
    """Literals: Strings, meaning words that are used in other files"""
    APP_CONFIG_SECTION_APP = 'application'
    APP_CONFIG_SECTION_ENTITY_DB = 'entity_db'
    APP_CONFIG_SECTION_ENTITY_API = 'entity_api'
    APP_CONFIG_SECTION_SHARED_LOG = 'shared_logger'
    APP_CONFIG_SECTION_SHARED_CACHE = 'shared_cache'

    APP_CONFIG_ROOT_DIR = '.'
    APP_CONFIG_FILE_PATH = '/resources/'
    APP_CONFIG_FILE_LOCATION = APP_CONFIG_ROOT_DIR + APP_CONFIG_FILE_PATH + 'app_config_dev.properties'
    APP_CONFIG_LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

    APP_SYS_USER_ID = '0'


class DBConnectionLiterals:
    DB_TYPE = 'mysql'
    DB_TYPE_DRIVER = 'pymysql'
    DB_SERVER = '172.16.1.173'
    DB_PORT = '3306'
    DB_NAME = 'iLead_prod_latest'
    DB_USER = 'ambarish'
    DB_HOST = DB_SERVER
    DB_PASSWORD = 'h8Fy^]xtK%Y5'


class FileLiterals:
    FILE_EXTN_PDF = 'pdf'
    FILE_EXTN_JSON = 'json'
    FILE_EXTN_TXT = 'txt'
    FILE_EXTN_XLS = 'xls'


class StringLiterals:
    STR_SEPARATOR_DOT = '.'
    STR_SEPARATOR_COLON = ':'
    STR_SEPARATOR_PIPE = '|'
    STR_SEPARATOR_AT_THE_RATE_OF = '@'
    STR_SEPARATOR_SLASH = '/'
    STR_SEPARATOR_SPACE = ' '
    STR_SEPARATOR_PLUS = '+'
    STR_EMPTY = ''
    STR_YES_FL = 'Y'
    STR_NO_FL = 'N'
    STR_TRUE = 'True'
    STR_FALSE = 'False'


class MessageLiterals:
    MSG_CONFIG_LOAD_FAILED = 'App Config load failed!'
    MSG_CONFIG_SECTION_LOAD_FAILED = 'App Config Section load failed!'
    MSG_LOGGER_INIT_FAILED = 'App Logger initialization failed!'
    MSG_REQUEST_JSON_NOT_VALID = 'Request JSON is not valid!'
    MSG_REPORT_SCHEDULING_FAILED = 'Report Scheduling Failed!'
    MSG_REPORT_SCHEDULING_SUCCESS = 'Report Scheduling Successfully Completed!'
    MSG_REPORT_GEN_FAILED = 'Report Generation Failed!'
    MSG_REPORT_GEN_SUCCESS = 'Report Generation Successfully Completed!'
    MSG_PROCESS_NOT_VALID = 'Process is not valid!'
    MSG_SCHEDULE_NOT_VALID = 'Process schedule is not valid!'

class DBConnectionLiterals:
    DB_TYPE = api_config_db['DB_TYPE']
    DB_TYPE_DRIVER = api_config_db['DB_TYPE_DRIVER']
    DB_SERVER = api_config_db['DB_SERVER']
    DB_PORT = api_config_db['DB_PORT']
    DB_NAME = api_config_db['DB_NAME']
    DB_USER = api_config_db['DB_USER']
    DB_HOST = DB_SERVER
    DB_PASSWORD = api_config_db['DB_PASSWORD']
