from scripts.byonic_data_access.db_connection_config import DBConnection


def login_user():
    try:
        DBSession = DBConnection().getsession()
        Query = DBSession.execute('select * from tbl_byonic_user').fetchall()
        return Query
    except:
        return False


if __name__ == '__main__':
    Data = login_user()
    print(Data)
