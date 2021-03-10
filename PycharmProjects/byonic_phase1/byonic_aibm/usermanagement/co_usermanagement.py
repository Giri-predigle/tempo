from sqlalchemy import Column, String, Integer, Date, Enum
from sqlalchemy.ext.declarative import declarative_base

from scripts.byonic_data_access.db_connection_config import DBConnection

ORMbase = declarative_base()


class COUser(ORMbase):
    __tablename__ = "tbl_byonic_user"

    col_user_id = Column('user_id', Integer, primary_key=True)
    col_user_first_name = Column('user_first_name', String, nullable=False)
    col_user_last_name = Column('user_last_name', String, nullable=False)
    col_user_email = Column('user_email', String, nullable=False)
    col_user_password = Column('user_password', String, nullable=False)
    col_user_role_id = Column('user_role_id', Integer, nullable=False)
    col_Status = Column('Status', Enum('Active', 'InActive'), nullable=False)
    col_reset = Column('reset', Enum('0', '1'), nullable=False)
    col_user_organisation_id = Column('user_organisation_id', Integer, nullable=False)
    col_last_login = Column('last_login', Date)


if __name__ == '__main__':
    print('Hello')
    # data = DBConnection().getsession()
    # user = data.query(COUser.col_user_id, COUser.col_user_first_name, COUser.col_user_last_name,
    #                   COUser.col_user_email, COUser.col_user_password, COUser.col_user_organisation_id,
    #                   COUser.col_user_role_id, COUser.col_Status, COUser.col_reset, COUser.col_last_login).all()
    #
    # values = ['id', 'firstname', 'lastname', 'email_addr', 'password', 'org_id',
    #           'role_id', 'Status', 'reset', 'last_login']
    # dictionary = []
    # for i in range(len(user)):
    #     files = dict(zip(values, user[i]))
    #     dictionary.append(files)
    # print(dictionary)
