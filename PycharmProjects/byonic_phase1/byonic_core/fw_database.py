import sys

from psycopg2.extras import DictCursor
from psycopg2.pool import SimpleConnectionPool
from sqlalchemy import create_engine
from sqlalchemy.pool import QueuePool
from sqlalchemy.orm import sessionmaker, scoped_session

from byonic_core.fw_config import CFAppConfig
from byonic_object.co_literal import CODBLiteral, COStringLiteral


class FWDBBase(object):
    pass


class FWDBSession(FWDBBase):

    __engine = None

    def __init__(self):
        if FWDBSession.__engine is None:
            config = CFAppConfig()
            conn_string = 'mysql://' + \
                          config.get_entity_db_user() + ':' + \
                          config.get_entity_db_password() + '@' + \
                          config.get_entity_db_host() + ':' + \
                          config.get_entity_db_port() + '/' + \
                          config.get_entity_db_name()
            # conn_string = 'mysql://byonic360:dbPa$$word360@3.12.207.166:3306/db_byonic'

            FWDBSession.__engine = create_engine(conn_string, echo=True, pool_size=int(CFAppConfig().get_pool_size())
                                                 , max_overflow=0, poolclass=QueuePool)

    @staticmethod
    def getsession():
        session = scoped_session(sessionmaker(bind=FWDBSession.__engine))
        return session()


# noinspection DuplicatedCode
class FWDBConnection(FWDBBase):
    """
    My Database objects, responsible for generating database connections , Connections in this class use connection
    pooling to obtain connection objects: conn = Mysql.getConn()
            Release connection objects;conn.close()or del conn
    """
    # Connection pool model
    __pool = None

    def __init__(self):
        # The database constructor takes the connection out of the connection pool and generates the operation cursor
        self._conn = FWDBConnection.__getconn()
        self._cursor = self._conn.cursor()

    @staticmethod
    def __getconn():
        """
        @summary: Static method to retrieve connection from connection pool
        @return FWDBConnection.connection
        """
        if FWDBConnection.__pool is None:
            # read connection parameters
            config = CFAppConfig()
            params = dict()
            params[CODBLiteral.DB_PARAM_HOST] = config.get_bot_db_host()
            params[CODBLiteral.DB_PARAM_PORT] = config.get_bot_db_port()
            params[CODBLiteral.DB_PARAM_DATABASE] = config.get_bot_db_name()
            params[CODBLiteral.DB_PARAM_USER] = config.get_bot_db_user()
            params[CODBLiteral.DB_PARAM_PASSWORD] = config.get_bot_db_password()

            FWDBConnection.__pool = SimpleConnectionPool(minconn=1, maxconn=5, cursor_factory=DictCursor, **params)
        return FWDBConnection.__pool.getconn()

    @staticmethod
    def __closeconns():
        """
        @summary: Static method to remove connections from connection pool
        @return FWDBConnection.connection
        """
        if FWDBConnection.__pool is not None:
            FWDBConnection.__pool.close_all()

    def exec_proc(self, name, param=None, refcursors=False):
        """
        @summary: Execute the query and fetch all result sets
        @param name: Name of the stored procedure or function to be executed
        @param param: Optional parameters, conditional list values (tuples)/List)
        @param refcursors: Optional parameters, indicates the presence of named cursors to retrieve resultset(s)
        @return: resultsets list of list(Dictionary model)/list(Dictionary model)/boolean Queried result sets
        """

        db_schema = CFAppConfig().get_bot_db_schema()
        name_w_schema = db_schema + COStringLiteral.STR_SEPARATOR_DOT + name
        sql = "Call " + name_w_schema
        if param is None:
            self._cursor.execute(sql)
        else:
            self._cursor.execute(sql, param)
        resultsets = dict()
        count = self._cursor.rowcount
        if count > 0:
            res = self._cursor.fetchall()
            if refcursors:
                named_cursors = res
                for named_cursor in named_cursors:
                    self._cursor.execute('FETCH ALL IN "' + named_cursor[0] + '"')
                    resultset = self._cursor.fetchall()
                    resultsets[named_cursor[0]] = resultset
            else:
                resultsets[0] = res
        return resultsets

    def exec_func(self, name, param=None, refcursors=False):
        """
        @summary: Execute the query and fetch all result sets
        @param name: Name of the stored procedure or function to be executed
        @param param: Optional parameters, conditional list values (tuples)/List)
        @param refcursors: Optional parameters, indicates the presence of named cursors to retrieve resultset(s)
        @return: resultsets list of list(Dictionary model)/list(Dictionary model)/boolean Queried result sets
        """

        db_schema = CFAppConfig().get_bot_db_schema()
        name = db_schema + COStringLiteral.STR_SEPARATOR_DOT + name
        if param is None:
            self._cursor.callproc(name)
        else:
            self._cursor.callproc(name, param)
        resultsets = dict()
        count = self._cursor.rowcount
        if count > 0:
            res = self._cursor.fetchall()
            if refcursors:
                named_cursors = res
                for named_cursor in named_cursors:
                    self._cursor.execute('FETCH ALL IN "' + named_cursor[0] + '"')
                    resultset = self._cursor.fetchall()
                    resultsets[named_cursor[0]] = resultset
            else:
                resultsets[0] = res
        return resultsets

    def getall(self, sql, param=None):
        """
        @summary: Execute the query and fetch all result sets
        @param sql:queryＳＱＬ，If there are query conditions, specify only the list of conditions and use parameters
        for the condition values[param]Pass in
        @param param: Optional parameters, conditional list values (tuples)/List)
        @return: result list(Dictionary model)/boolean Queried result sets
        """
        res = False
        if param is None:
            self._cursor.execute(sql)
        else:
            self._cursor.execute(sql, param)
        count = self._cursor.rowcount
        if count > 0:
            res = self._cursor.fetchall()
        return res

    def getone(self, sql, param=None):
        """
        @summary: Execute the query and take out Article 1
        @param sql:queryＳＱＬ，If there are query conditions, specify only the list of conditions and use parameters
        for the condition values[param]Pass in
        @param param: Optional parameters, conditional list values (tuples)/List)
        @return: result list/boolean Queried result sets
        """
        res = False
        if param is None:
            self._cursor.execute(sql)
        else:
            self._cursor.execute(sql, param)
        count = self._cursor.rowcount
        if count > 0:
            res = self._cursor.fetchone()
        return res

    def getmany(self, sql, num, param=None):
        """
        @summary: Execute the query and take it out num Article result
        @param sql:queryＳＱＬ，If there are query conditions, specify only the list of conditions and use parameter
        for the condition values[param]Pass in
        @param num:Number of results obtained
        @param param: Optional parameters, conditional list values (tuples)/List)
        @return: result list/boolean Queried result sets
        """
        res = False
        if param is None:
            self._cursor.execute(sql)
        else:
            self._cursor.execute(sql, param)
        count = self._cursor.rowcount
        if count > 0:
            res = self._cursor.fetchmany(num)
        return res

    def insertone(self, sql, value):
        """
        @summary: Insert a record into the data table
        @param sql:To insertＳＱＬformat
        @param value:Record data to be inserted tuple/list
        @return: insertId Number of rows affected
        """
        self._cursor.execute(sql, value)
        insert_id = self._cursor.fetchone()[0]
        return insert_id

    def insertmany(self, sql, values):
        """
        @summary: Insert multiple records into the data table
        @param sql:To insertＳＱＬformat
        @param values:Record data to be inserted tuple(tuple)/list[list]
        @return: count Number of rows affected
        """
        self._cursor.executemany(sql, values)
        count = self._cursor.rowcount
        return count

    def __query(self, sql, param=None):
        if param is None:
            self._cursor.execute(sql)
        else:
            self._cursor.execute(sql, param)
        count = self._cursor.rowcount
        return count

    def update(self, sql, param=None):
        """
        @summary: Update data table records
        @param sql: ＳＱＬFormat and conditions, use(%s,%s)
        @param param: To be updated  value tuple/list
        @return: count Number of rows affected
        """
        return self.__query(sql, param)

    def delete(self, sql, param=None):
        """
        @summary: Delete data table records
        @param sql: ＳＱＬFormat and conditions, use(%s,%s)
        @param param: Conditions to be deleted value tuple/list
        @return: count Number of rows affected
        """
        return self.__query(sql, param)

    def begin(self):
        """
        @summary: Open a transaction
        """
        self._conn.autocommit = True

    def end(self, option=CODBLiteral.DB_COMMIT):
        """
        @summary: Closing the transaction
        """
        if option == CODBLiteral.DB_COMMIT:
            self._conn.commit()
        else:
            self._conn.rollback()

    def dispose(self, is_end=CODBLiteral.DB_SUCCESS):
        """
        @summary: Release connection pool resources
        """
        if is_end == CODBLiteral.DB_SUCCESS:
            self.end(CODBLiteral.DB_COMMIT)
        else:
            self.end(CODBLiteral.DB_ROLLBACK)
        self._cursor.close()
        self._conn.close()


if __name__ == '__main__':
    SELECT_QUERY = 'select version()'
    conn_pool = FWDBConnection()
    status = CODBLiteral.DB_SUCCESS
    try:
        result = conn_pool.getone(SELECT_QUERY)
        if result is None:
            raise ValueError
        else:
            print(result)
    except ValueError:
        status = CODBLiteral.DB_FAILURE
        print(sys.exc_info()[0])
        raise
    finally:
        conn_pool.dispose(status)
