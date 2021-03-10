from sqlalchemy import create_engine, MetaData
from sqlalchemy.orm import sessionmaker, scoped_session
from scripts.byonic_data_access.db_config import DBDetails
from scripts.byonic_data_access.literal_config import StringLiterals


class FWDBBase(object):
    pass


class DBConnection(FWDBBase):
    __engine = None

    def __init__(self):
        if DBConnection.__engine is None:
            db_config = DBDetails()
            string_literal = StringLiterals()

            self.conn_string = db_config.get_entity_db_type() + \
                               string_literal.STR_SEPARATOR_PLUS + \
                               db_config.get_entity_db_type_driver() + \
                               string_literal.STR_SEPARATOR_COLON + \
                               string_literal.STR_SEPARATOR_SLASH * 2 + \
                               db_config.get_entity_db_user() + \
                               string_literal.STR_SEPARATOR_COLON + \
                               db_config.get_entity_db_password() + \
                               string_literal.STR_SEPARATOR_AT_THE_RATE_OF + \
                               db_config.get_entity_db_host() + \
                               string_literal.STR_SEPARATOR_COLON + \
                               db_config.get_entity_db_port() + \
                               string_literal.STR_SEPARATOR_SLASH + \
                               db_config.get_entity_db_name()
            print(self.conn_string)

            # metadata = MetaData()
            # userdb = Table('tbl_byonic_user', metadata, autoload=True, autoload_with=DBConnection.__engine)
            # print(userdb.columns.keys())
            DBConnection.__engine = create_engine(self.conn_string, echo=True)

    def getEngine(self):
        DBConnection.__engine = create_engine(self.conn_string, echo=True)
        conn = DBConnection.__engine.connect()
        return conn

    @staticmethod
    def getsession():
        Session = sessionmaker()
        session = scoped_session(sessionmaker(bind=DBConnection.__engine))
        return session()

    @staticmethod
    def metadata():
        metadata = MetaData()
        return metadata


if __name__ == '__main__':
    DBConnection().getEngine()
