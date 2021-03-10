from scripts.byonic_data_access.literal_config import DBConnectionLiterals


class DBDetails:
    def __init__(self):
        """
        DB Details
        """
        self.DB_TYPE = DBConnectionLiterals.DB_TYPE
        self.DB_TYPE_DRIVER = DBConnectionLiterals.DB_TYPE_DRIVER
        self.DB_SERVER = DBConnectionLiterals.DB_SERVER
        self.DB_PORT = DBConnectionLiterals.DB_PORT
        self.DB_NAME = DBConnectionLiterals.DB_NAME
        self.DB_USER = DBConnectionLiterals.DB_USER
        self.DB_HOST = DBConnectionLiterals.DB_HOST
        self.DB_PASSWORD = DBConnectionLiterals.DB_PASSWORD
        return

    def get_entity_db_type(self):
        db_type = self.DB_TYPE
        return db_type

    def get_entity_db_type_driver(self):
        db_type_driver = self.DB_TYPE_DRIVER
        return db_type_driver

    def get_entity_db_server(self):
        db_server = self.DB_SERVER
        return db_server

    def get_entity_db_port(self):
        db_port = self.DB_PORT
        return db_port

    def get_entity_db_name(self):
        db_name = self.DB_NAME
        return db_name

    def get_entity_db_user(self):
        db_user = self.DB_USER
        return db_user

    def get_entity_db_host(self):
        db_host = self.DB_HOST
        return db_host

    def get_entity_db_password(self):
        db_password = self.DB_PASSWORD
        return db_password


if __name__ == "__main__":
    DBDetails().get_entity_db_type()
