from scripts.byonic_data_access.db_connection_config import DBConnection
from byonic_aibm.usermanagement.co_usermanagement import COUser


class DAUser:
    def __init__(self):
        return

    @staticmethod
    def load_all_user():
        data = DBConnection().getsession()
        user = data.query(COUser.col_user_id, COUser.col_user_first_name, COUser.col_user_last_name,
                          COUser.col_user_email, COUser.col_user_password, COUser.col_user_organisation_id,
                          COUser.col_user_role_id, COUser.col_Status, COUser.col_reset, COUser.col_last_login).all()

        values = ['id', 'firstname', 'lastname', 'email_addr', 'password', 'org_id',
                  'role_id', 'Status', 'reset', 'last_login']
        dictionary = []
        for i in range(len(user)):
            files = dict(zip(values, user[i]))
            dictionary.append(files)
        print(dictionary)

        def edit_user():
            pass

        def delete_user():
            pass


if __name__ == "__main__":
    DAUser().load_all_user()
