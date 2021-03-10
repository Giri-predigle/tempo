from byonic_object.co_base import COBase


class COAppLiteral(COBase):
    APP_CONFIG_SECTION_APP = 'application'
    APP_CONFIG_SECTION_ENTITY_DB = 'entity_db'
    APP_CONFIG_SECTION_ENTITY_API = 'byonic_api'
    APP_CONFIG_SECTION_SHARED_LOG = 'shared_logger'
    APP_CONFIG_SECTION_SHARED_CACHE = 'shared_cache'
    APP_CONFIG_SECTION_POOLING = 'pool_size'
    APP_CONFIG_SECTION_FTP = 'ftp_access'

    APP_CONFIG_ROOT_DIR = '.'
    APP_CONFIG_FILE_PATH = '/resources/'
    APP_CONFIG_FILE_LOCATION = APP_CONFIG_ROOT_DIR + APP_CONFIG_FILE_PATH + 'app_config_dev.properties'
    APP_CONFIG_LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

    APP_SYS_USER_ID = '0'


class COPropLiteral(COBase):
    PROP_PREFIX = 'byonic.'

    PROP_APP_NAME = PROP_PREFIX + 'app.name'
    PROP_APP_VERSION = PROP_PREFIX + 'app.version'
    PROP_APP_INSTANCE = PROP_PREFIX + 'app.instance'
    PROP_APP_SYS_USER = PROP_PREFIX + 'app.sysuser'

    PROP_ENTITY_DB_NAME = PROP_PREFIX + 'entity.db.name'
    PROP_ENTITY_DB_ALIAS = PROP_PREFIX + 'entity.db.alias'
    PROP_ENTITY_DB_HOST = PROP_PREFIX + 'entity.db.host'
    PROP_ENTITY_DB_PORT = PROP_PREFIX + 'entity.db.port'
    PROP_ENTITY_DB_USERNAME = PROP_PREFIX + 'entity.db.username'
    PROP_ENTITY_DB_PASSWORD = PROP_PREFIX + 'entity.db.password'

    PROP_ENTITY_API_NAME = PROP_PREFIX + 'aibm.api.name'
    PROP_ENTITY_API_PORT = PROP_PREFIX + 'aibm.api.port'

    PROP_POOL_SIZE = PROP_PREFIX + 'pool.size'

    PROP_SHARED_LOG_FILENAME = PROP_PREFIX + 'shared.log.filename'
    PROP_SHARED_LOG_LEVEL = PROP_PREFIX + 'shared.log.level'
    PROP_SHARED_DOCUMENT_FOLDER = PROP_PREFIX + 'shared.document.folder'
    PROP_SHARED_RFP_DOCUMENT_PREFIX = PROP_PREFIX + 'shared.document.rfp.prefix'
    PROP_SHARED_REPORT_FOLDER = PROP_PREFIX + 'shared.report.folder'
    PROP_SHARED_REPORT_MIS_PREFIX = PROP_PREFIX + 'shared.report.mis.prefix'
    PROP_SHARED_PAGE_ROWS_DEFAULT = PROP_PREFIX + 'shared.page.rows.default'
    PROP_SHARED_PAGE_COUNT_DEFAULT = PROP_PREFIX + 'shared.page.count.default'

    PROP_SHARED_CACHE_ENABLED = PROP_PREFIX + 'shared.cache.enabled'


class CODBLiteral(COBase):
    # DB Config Literals
    DB_PARAM_HOST = 'host'
    DB_PARAM_PORT = 'port'
    DB_PARAM_DATABASE = 'database'
    DB_PARAM_USER = 'user'
    DB_PARAM_PASSWORD = 'password'
    DB_PARAM_NEW_ID = -1

    # DB Constant Literals
    DB_SUCCESS = 1
    DB_FAILURE = 0
    DB_COMMIT = 'commit'
    DB_ROLLBACK = 'rollback'


class COEnumLiteral(COBase):
    # REPORT run status
    EN_REPORT_PENDING = 'Pending'
    EN_REPORT_IN_PROGRESS = 'In-Progress'
    EN_REPORT_COMPLETED = 'Completed'
    EN_REPORT_ABORTED = 'Aborted'

    EN_STATUS_CREATED = 'Created'
    EN_STATUS_LIVE = 'Live'
    EN_STATUS_SCHEDULING = 'Scheduling'
    EN_STATUS_SCHEDULED = 'Scheduled'
    EN_STATUS_SCHEDULING_FAILED = 'SchedulingFailed'

    # Schedule Frequency Types
    EN_SCHEDULE_DAILY = 'D'
    EN_SCHEDULE_WEEKLY = 'W'
    EN_SCHEDULE_MONTHLY = 'M'
    EN_SCHEDULE_YEARLY = 'Y'
    EN_SCHEDULE_FIXED = 'F'


class COFileLiteral(COBase):
    FILE_EXTN_PDF = 'pdf'
    FILE_EXTN_JSON = 'json'
    FILE_EXTN_TXT = 'txt'
    FILE_EXTN_XLS = 'xls'


class COStringLiteral(COBase):
    STR_SEPARATOR_DOT = '.'
    STR_SEPARATOR_COLON = ':'
    STR_SEPARATOR_PIPE = '|'
    STR_SEPARATOR_SLASH = '/'
    STR_SEPARATOR_SPACE = ' '
    STR_EMPTY = ''
    STR_YES_FL = 'Y'
    STR_NO_FL = 'N'
    STR_TRUE = 'True'
    STR_FALSE = 'False'


class COKeyLiteral(COBase):
    KEY_OBJECT_PREFIX = 'OBJECT_KEY' + COStringLiteral.STR_SEPARATOR_PIPE
    KEY_OBJECT_REPORT_ALL = KEY_OBJECT_PREFIX + 'REPORT_ALL'
    KEY_OBJECT_REPORT_SCHEDULE_ALL = KEY_OBJECT_PREFIX + 'REPORT_SCHEDULE_ALL'


class COMessageLiteral(COBase):
    # Framework Messages
    MSG_CONFIG_LOAD_FAILED = 'App Config load failed!'
    MSG_CONFIG_SECTION_LOAD_FAILED = 'App Config Section load failed!'
    MSG_LOGGER_INIT_FAILED = 'App Logger initialization failed!'
    MSG_REQUEST_JSON_NOT_VALID = 'Request JSON is not valid!'
    MSG_PROCESS_NOT_VALID = 'Process is not valid!'
    MSG_SCHEDULE_NOT_VALID = 'Process schedule is not valid!'
