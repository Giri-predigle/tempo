from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, scoped_session
from scripts.DB_config import Db_config
api_config_db = Db_config()


class FWDBBase(object):
    pass


class FWDBSession(FWDBBase):

    __engine = None

    def __init__(self):
        if FWDBSession.__engine is None:
            conn_string = 'mysql://' + \
                          api_config_db['USERNAME'] + ':' + \
                          api_config_db['PASSWORD'] + '@' + \
                          api_config_db['HOST'] + ':' + \
                          api_config_db['PORT'] + '/' + \
                          api_config_db['DATABASE']
            # conn_string = 'mysql://ilead360:dbPa$$word360@3.12.207.166:3306/db_ilead'
            FWDBSession.__engine = create_engine(conn_string, echo=True)

    @staticmethod
    def getsession():
        session = scoped_session(sessionmaker(bind=FWDBSession.__engine))
        return session()

if __name__ == "__main__":
    connection = FWDBSession()