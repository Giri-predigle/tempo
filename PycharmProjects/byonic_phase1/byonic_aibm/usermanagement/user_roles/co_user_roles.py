from sqlalchemy import Column, String, Integer, Date, Enum
from sqlalchemy.ext.declarative import declarative_base

ORMbase = declarative_base()


class COUserRoles(ORMbase):
    __tablename__ = "tbl_byonic_roles"

    col_user_role_id = Column('user_role_id', Integer, primary_key=True)
    col_user_role = Column('user_role', String, nullable=False)