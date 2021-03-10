from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_aibm.usermanagement.user_roles.co_user_roles import COUserRoles


class DAUserRoles:
    def __init__(self):
        return

    @staticmethod
    def load_all_user_roles():
        data = DBConnection().getsession()
        user_roles = data.query(COUserRoles.col_user_role).all()
        print(user_roles)


if __name__ == "__main__":
    DAUserRoles().load_all_user_roles()