import os
import json

config_file = os.getcwd() + '/config/config_file.json'
print(config_file)


def Db_config():
    with open(config_file, 'r') as file:
        api_config = json.load(file)
        api_config_db = api_config['Database']
    file.close()
    api_config_de = api_config_db['db_ilead_Second_dev']
    # api_config_de = api_config_db['db_byonic_uat']
    return api_config_de


def get_port():
    """ This function used to get the port number on which the flask app will run.
    :return: set_port
    """
    # since we are declaring all var as global in variables() we need to call variables() to access

    with open(config_file, 'r') as file:
        api_config_details = json.load(file)
        current_region = api_config_details['Region']
        api_config_details = api_config_details['EnvPorts']
        file.close()

    set_port = api_config_details[current_region]
    # print(set_port)
    return set_port


def get_login_config():
    """ This function used to get the port number on which the flask app will run.
    :return: set_port
    """
    # since we are declaring all var as global in variables() we need to call variables() to access

    with open(config_file, 'r') as file:
        api_configuration_details = json.load(file)
        secretKey = api_configuration_details['SECRET_KEY']
        expiryTime = api_configuration_details['EXPIRY_TIME']
        file.close()

    return secretKey, expiryTime


if __name__ == '__main__':
    get_port()
