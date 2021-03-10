"""
    created on: 8/20/20
"""
import json
from byonic_aibm.usermanagement.byonic_user.rs_byonic_user import *
from datetime import date
import time
from scripts import filter_model_list
import logging
import hashlib
import redis
from flask import Flask, jsonify, request, Response
from flask_cors import CORS, cross_origin

from dateutil.relativedelta import relativedelta
from scripts import response
import os
from pathlib import Path
from scripts import userdata_mgmt as user_data
from scripts import login as logincheck
from scripts import companyCard as companyCard
from scripts.resetpassword import resetPassword
from byonic_aibm.usermanagement.user_role.rs_user_role import *
from byonic_aibm.domain_filtering import da_domain_filtering
from byonic_aibm.side_filters import bl_side_filters
from byonic_core.fw_config import CFAppConfig
from byonic_core.fw_auth import HTTPBasicAuth
from byonic_aibm.side_filters.country.rs_country import *
from byonic_aibm.side_filters.intent_topics.rs_intent_topics import *
from byonic_aibm.side_filters.job_levels.rs_job_level import *
from byonic_aibm.side_filters.employee_size.rs_employee_size import *
from byonic_aibm.side_filters.industry.rs_industry import *
from byonic_aibm.side_filters.revenue_range.rs_revenue_range import *

config = CFAppConfig()
auth = HTTPBasicAuth()

APP = Flask(config.get_aibm_api_name())
cors = CORS(APP, resources={r"/api/*": {"origins": "*"}})
APP.config['CORS_HEADERS'] = 'Content-Type'
r = redis.StrictRedis(host='localhost', port=6379, db=0)

global database, config_file, data_path, topic_json_path, configuration_file
global industry_json_path, country_json_path, job_json_path, cwd
global emp_size_path, CURRENT_WORKING_DIR, debug_mode, intent_request
global req_country, req_job, req_employee, req_topic, req_industry, req_jobtitle, req_revenue, req_domainEx, req_domainIn
global filename, lastname, email_addr, password, id, role, status


def use_global_variables():
    """
    initialize global variables
    :return:
    """

    global database, config_file, data_path, topic_json_path, configuration_file
    global industry_json_path, country_json_path, job_json_path, cwd
    global emp_size_path, CURRENT_WORKING_DIR, debug_mode, intent_request
    global filename, lastname, email_addr, password, id, role

    # model = response.Database()
    # database = model.database
    model = bl_side_filters.Sidefilters()
    database = model

    CURRENT_WORKING_DIR = Path.cwd()
    cwd = os.getcwd()
    config_file = cwd + '/config/config.json'

    with open(config_file, 'r') as file:
        api_config = json.load(file)
        api_config_details = api_config['Path']
    file.close()

    data_path = api_config_details['API_JSON_PATH']

    if 'DebugMode' in api_config and api_config['DebugMode'].upper() == "TRUE":
        debug_mode = True
    else:
        debug_mode = False


def check_redis_connection():
    try:
        resp = r.client_list()
        return resp
    except redis.ConnectionError:
        print('Error Connecting to Redis Server')
        return False


def get_login_config():
    """ This function used to get the port number on which the flask app will run.
    :return: set_port
    """

    use_global_variables()
    # since we are declaring all var as global in variables() we need to call variables() to access

    with open(config_file, 'r') as file:
        api_configuration_details = json.load(file)
        secretKey = api_configuration_details['SECRET_KEY']
        expiryTime = api_configuration_details['EXPIRY_TIME']
        file.close()

    return secretKey, expiryTime


def process_request(req_content):
    global req_country, req_job, req_employee, req_topic, req_industry, req_jobtitle, req_revenue, req_domain, \
        req_domainEx, req_domainIn
    # req_intentgrade
    resp_content = req_content

    if 'industries' in req_content:
        req_industry = req_content.get('industries')
    industry = req_industry
    if 'topics' in req_content:
        req_topic = req_content.get('topics')
    topic = req_topic
    if "employee_sizes" in req_content:
        req_employee = req_content.get("employee_sizes")
    employee = req_employee
    if "job_levels" in req_content:
        req_job = req_content.get('job_levels')
    job = req_job
    if "countries" in req_content:
        req_country = req_content.get('countries')
    country = req_country
    # if "job_titles" in req_content:
    #     req_jobtitle = req_content.get('job_titles')
    # jobtitle = req_jobtitle
    if "revenues" in req_content:
        req_revenue = req_content.get('revenues')
    revenue = req_revenue
    if "domains" in req_content:
        req_domain = req_content.get('domains')
    domain = req_domain
    if "domainEx" in req_content:
        req_domainEx = req_content.get('domainEx')
    domainEx = req_domainEx
    if "domainIn" in req_content:
        req_domainIn = req_content.get('domainIn')
    domainIn = req_domainIn

    if "intentgrades" in req_content:
        req_intentgrade = req_content.get('intentgrades')
    intentgrade = req_intentgrade

    # model = filter_model_list.MLModel(req_industry=industry, req_topic=topic, req_employee=employee,
    #                                   req_job=job, req_country=country, req_jobtitle=jobtitle, req_revenue=revenue,
    #                                   req_domain=domain,
    #                                   req_intentgrade=intentgrade
    #                                   )
    model = da_domain_filtering.DADomain_Filter(req_industry=industry, req_topic=topic, req_employee=employee,
                                                req_job=job, req_country=country,
                                                req_revenue=revenue,
                                                req_domain=domain,
                                                req_intentgrade=intentgrade,
                                                req_domainEx=domainEx,
                                                req_domainIn=domainIn
                                                )
    resp_data = model.load_all_domain()

    if 'Error' in resp_data:
        resp_content['Error'] = resp_data['Error']
        resp_content['Status'] = "Failed"
    else:
        if 'Domains' in resp_data:
            resp_content['Domains'] = resp_data['Domains']

    return resp_content


@APP.route('/api/intent/signal', methods=['POST'])
@cross_origin()
def worker():
    """
    api call start
    """

    get_date = time.strftime("%Y%m%d")
    fname = Path(os.path.dirname(os.path.abspath(__file__)) + "/logs/api/api" + get_date + '.log')
    logging.basicConfig(filename=fname, level=logging.INFO)
    logger = logging.getLogger()
    logger.info(":::::::Post Method :::::::")
    logger.info('Request came in')

    if request.data:
        redis_okay = check_redis_connection()
        req_content = request.get_json()
        print('req_content', req_content)
        no_of_requests = len(req_content.get('Requests'))
        logger.info("Number of request to process:{0}".format(no_of_requests + 1))
        resp = {}
        ifuture = {}
        # calculated_future = {}
        for index in range(no_of_requests):
            logger.info("processing request no: {0}".format(index))
            current_request = req_content.get('Requests')[index]
            logger.info("current request : {0}".format(current_request))

            if redis_okay:
                today = date.today()
                last_month_end = date(today.year, today.month, 1) - relativedelta(days=1)
                last_month_end = last_month_end.strftime("%Y%m%d")
                request_json_dump = json.dumps(current_request)
                r_key = "predigleai_" + str(last_month_end) + "_" + hashlib.md5(
                    request_json_dump.encode("utf-8")).hexdigest()
                logger.info('Request Hash => {}'.format(r_key))
                logger.info('Redis Exists => {}'.format(r.exists(r_key)))
                if r.exists(r_key) == 1:
                    temp = json.loads(r.get(r_key).decode('utf-8'))
                    print('TEMP-1 => {}'.format(temp))
                else:
                    temp = process_request(current_request)

            else:
                temp = process_request(current_request)
            if 'Error' not in temp:
                resp[index] = temp
                logger.info("request ran successfully")
                ifuture['industries'] = temp['industries']
                ifuture['topics'] = temp['topics']
                ifuture['job_levels'] = temp['job_levels']
                ifuture["employee_sizes"] = temp["employee_sizes"]
                ifuture['countries'] = temp['countries']
                # ifuture['job_titles'] = temp['job_titles']
                ifuture['revenues'] = temp['revenues']
                # ifuture['intentgrades'] = temp['intentgrades']
            else:
                json_object = json.dumps(temp, ensure_ascii=False).encode('utf8')
                resp = Response(json_object, status=500, mimetype='Application/json')
                # ks.backend.clear_session()
                resp.headers.add('Access-Control-Allow-Origin', '*')
                print('Early:', resp)
                return resp

        # logger.info('NUM REQUESTS => ', no_of_requests)
        resp_data = resp[0]
    else:
        resp_data = {"Error": " Request JSON is empty or has invalid data "}

    resp_content = resp_data

    if 'Error' in resp_data:
        json_object = json.dumps(resp_content, ensure_ascii=False).encode('utf8')
        resp = Response(json_object, status=500, mimetype='Application/Json')
    else:
        json_object = json.dumps(resp_content, ensure_ascii=False).encode('utf8')
        resp = Response(json_object, mimetype='Application/json')

    return resp


@APP.route('/api/sidenav', methods=['GET'])
@cross_origin()
def api_side():
    """
    API for accessing full database from ilead
    :return:
    """
    if request.method == 'GET':
        countries = get_countries_by_name()
        industries = get_industries_by_name()
        revenue = get_revenue_range_values_by_range()
        job_levels = get_job_levels_by_name()
        employee_size = get_employee_sizes_by_value()
        topics = get_topics_by_name()

        resp = {key: topics.get(key, [])
                     + job_levels.get(key, [])
                     + employee_size.get(key, [])
                     + countries.get(key, [])
                     + industries.get(key, [])
                     + revenue.get(key, [])
                for key in set(list(topics.keys()) + list(job_levels.keys())
                               + list(employee_size.keys()) + list(countries.keys())
                               + list(industries.keys()) + list(revenue.keys()))}

        return jsonify(resp)


@APP.route('/api/sidenav/topics', methods=['GET'])
@cross_origin()
def api_topic():
    """
    API for accessing Topics from ilead database
    :return:
    """
    if request.method == "GET":
        return jsonify(get_topics_by_name())


@APP.route('/api/sidenav/industries', methods=['GET'])
@cross_origin()
def api_industry():
    """
    API for accessing Industries from ilead database
    :return:
    """
    if request.method == "GET":
        return jsonify(get_industries_by_name())


@APP.route('/api/sidenav/countries', methods=['GET'])
@cross_origin()
def api_country():
    """
    API for accessing countries from ilead database
    :return:
    """
    if request.method == "GET":
        return jsonify(get_countries_by_name())


@APP.route('/api/sidenav/joblevels', methods=['GET'])
@cross_origin()
def api_job():
    """
    API for accessing job_level from ilead database
    :return:
    """
    if request.method == "GET":
        return jsonify(get_job_levels_by_name())


@APP.route('/api/sidenav/emp_sizes', methods=['GET'])
@cross_origin()
def api_emp():
    """
    API for accessing employee_size from ilead database
    :return:
    """
    if request.method == 'GET':
        return jsonify(get_employee_sizes_by_value())


@APP.route('/api/sidenav/revenue', methods=['GET'])
@cross_origin()
def api_revenue():
    """
    API for accessing Topics from ilead database
    :return:
    """
    if request.method == 'GET':
        return jsonify(get_revenue_range_values_by_range())

# @APP.route('/api/sidenav/job_titles', methods=['GET'])
# @cross_origin()
# def api_jobtitles():
#     """
#     API for accessing job_titles from ilead database
#     :return:
#     """
#     use_global_variables()
#     # since we are declaring all var as global in variables() we need to call variables() to access
#     get_date = time.strftime("%Y%m%d")
#     file_name = Path(os.path.dirname(os.path.abspath(__file__)) + "/logs/api/api" + get_date + '.log')
#     logging.basicConfig(filename=file_name, level=logging.INFO)
#     logger = logging.getLogger()  # getting logger information
#     logger.info(":::::::GET Method :::::::")
#
#     redis_okay = check_redis_connection()  # checking redis connection
#     books = {'job_titles': database['job_titles']}  # database connectivity
#     resp_db = books
#
#     if redis_okay:
#         today = date.today()
#         last_month_end = date(today.year, today.month, today.day)
#         last_month_end = last_month_end.strftime("%Y%m%d")
#         request_json_dump = json.dumps(resp_db)
#         r_key = "job_titles-" + str(last_month_end) + "-" + hashlib.md5(request_json_dump.encode("utf-8")).hexdigest()
#         # redis key
#         logger.info('job_titles')
#         logger.info('Request Hash => {}'.format(r_key))
#         logger.info('Redis Exists => {}'.format(r.exists(r_key)))
#
#         if r.exists(r_key) == 1:
#             temp = json.loads(r.get(r_key).decode('utf-8'))
#         else:
#             temp = resp_db
#         if 'Error' not in temp:
#             if redis_okay:
#                 r_value = json.dumps(temp)
#                 r.set(r_key, r_value)
#             logger.info("request ran successfully")
#
#         # if debug_mode:
#         #     get_time = time.strftime("%Y%m%d-%H%M%S")
#         #     output_json_list = "sidenav_response_job_titles" + get_time + ".json"
#         #     out_json_name = cwd + "/{}/{}".format(topic_json_path, output_json_list)
#         #
#         #     # with open(out_json_name, 'w', encoding='utf8') as jsl:
#         #     #     json.dump(resp_db, jsl, indent=4)
#         #     #     jsl.close()
#
#     return jsonify(resp_db)


# @APP.route('/api/user', methods=['GET'])
@cross_origin()
def user_management():
    user = user_data.Userdatabase()
    user_db = user.db_loader_user
    use_global_variables()
    # since we are declaring all var as global in variables() we need to call variables() to access

    get_date = time.strftime("%Y%m%d")
    file_name = Path(os.path.dirname(os.path.abspath(__file__)) + "/logs/api/api" + get_date + '.log')
    logging.basicConfig(filename=file_name, level=logging.INFO)
    logger = logging.getLogger()  # getting logger information
    logger.info(":::::::GET Method :::::::")

    redis_okay = check_redis_connection()  # checking redis connection
    resp_db = user_db  # database connectivity
    # resp = {}

    if redis_okay:
        today = date.today()
        last_month_end = date(today.year, today.month, today.day)
        last_month_end = last_month_end.strftime("%Y%m%d")
        request_json_dump = json.dumps(resp_db)
        r_key = "predigle-" + str(last_month_end) + "-" + hashlib.md5(request_json_dump.encode("utf-8")).hexdigest()
        # redis key
        logger.info('All')
        logger.info('Request Hash => {}'.format(r_key))
        logger.info('Redis Exists => {}'.format(r.exists(r_key)))
        if r.exists(r_key) == 1:
            temp = json.loads(r.get(r_key).decode('utf-8'))

        else:
            temp = resp_db

        if 'Error' not in temp:
            if redis_okay:
                r_value = json.dumps(temp)
                r.set(r_key, r_value)
            resp = temp  # database
            logger.info("request ran successfully")

        if debug_mode:
            get_time = time.strftime("%Y%m%d-%H%M%S")
            output_json_list = "user_response-" + get_time + ".json"
            out_json_name = cwd + "/{}/{}".format(data_path, output_json_list)

            # with open(out_json_name, 'w', encoding='utf8') as jsl:
            #     json.dump(resp, jsl, indent=4)
            #     jsl.close()

    return jsonify(resp_db)
    #
    # return jsonify(user_db)


# @APP.route('/api/user/<identifier>', methods=['GET'])
@cross_origin()
def Adminuser_management(identifier):
    org_id = identifier
    firstname = None
    lastname = None
    email_addr = None
    status = None
    role_id = None

    id = None
    password = None
    user = user_data.Userdatabase(firstname, lastname, email_addr, id, password, status, role_id, org_id)
    user_db = user.db_loader_user
    ################################################################################
    use_global_variables()
    # since we are declaring all var as global in variables() we need to call variables() to access

    get_date = time.strftime("%Y%m%d")
    file_name = Path(os.path.dirname(os.path.abspath(__file__)) + "/logs/api/api" + get_date + '.log')
    logging.basicConfig(filename=file_name, level=logging.INFO)
    logger = logging.getLogger()  # getting logger information
    logger.info(":::::::GET Method :::::::")

    redis_okay = check_redis_connection()  # checking redis connection
    resp_db = user_db  # database connectivity
    resp = {}

    if redis_okay:
        today = date.today()
        last_month_end = date(today.year, today.month, today.day)
        last_month_end = last_month_end.strftime("%Y%m%d")
        request_json_dump = json.dumps(resp_db)
        r_key = "predigle-" + str(last_month_end) + "-" + hashlib.md5(request_json_dump.encode("utf-8")).hexdigest()
        # redis key
        logger.info('All')
        logger.info('Request Hash => {}'.format(r_key))
        logger.info('Redis Exists => {}'.format(r.exists(r_key)))
        if r.exists(r_key) == 1:
            temp = json.loads(r.get(r_key).decode('utf-8'))

        else:
            temp = resp_db

        if 'Error' not in temp:
            if redis_okay:
                r_value = json.dumps(temp)
                r.set(r_key, r_value)
            # resp = temp  # database
            logger.info("request ran successfully")

        # if debug_mode:
        #     get_time = time.strftime("%Y%m%d-%H%M%S")
        #     output_json_list = "user_response-" + get_time + ".json"
        #     out_json_name = cwd + "/{}/{}".format(data_path, output_json_list)
        #
        #     # with open(out_json_name, 'w', encoding='utf8') as jsl:
        #     #     json.dump(resp, jsl, indent=4)
        #     #     jsl.close()

    return jsonify(resp_db)


# @APP.route('/api/user/update/<identifier>', methods=['POST'])
@cross_origin()
def user_update(identifier):
    content = request.get_json()
    # print(content)
    firstname = content.get('firstname')
    lastname = content.get('lastname')
    email_addr = content.get('email')
    status = content.get('status')
    role = content.get('role')
    org = content.get('organisation')
    id = identifier
    password = content.get('password')
    user = user_data.Userdatabase(firstname, lastname, email_addr, id, password, status, role, org)
    user_db = user.UPDATE_user_db

    return jsonify('UPDATE SUCCESSFUL')


# @APP.route('/api/user/insert', methods=['POST'])
@cross_origin()
def user_insert():
    content = request.get_json()
    # print(content)
    firstname = content.get('firstname')
    lastname = content.get('lastname')
    email_addr = content.get('email')
    id = content.get('id')
    status = content.get('status')
    password = content.get('password')
    role_id = content.get('role')
    org_id = content.get('organisation')

    user = user_data.Userdatabase(firstname, lastname, email_addr, id, password, status, role_id, org_id)
    user_db = user.INSERT_user_db
    return jsonify('Added Successful')


# @APP.route('/api/user/delete/<identifier>', methods=['DELETE'])
@cross_origin()
def user_delete(identifier):
    id = identifier
    firstname = None
    lastname = None
    email_addr = None
    password = None
    status = None
    # print(id)

    user = user_data.Userdatabase(firstname, lastname, email_addr, id, password, status, role_id=None, org_id=None)
    user_db = user.DELETE_user_db
    return jsonify("successfully deleted")


@APP.route('/api/userrole', methods=['GET'])
@cross_origin()
def role_view():
    role = user_data.Role_db()
    role_db = role.View_role

    return jsonify(role_db)


@APP.route('/api/organisation', methods=['GET'])
@cross_origin()
def org_view():
    org = user_data.Organisation_db()
    org_db = org.View_organisation

    return jsonify(org_db)


@APP.route('/api/organisation/insert', methods=['POST'])
@cross_origin()
def org_insert():
    content = request.get_json()
    org_name = content.get('organisation')

    org = user_data.Organisation_db(org_name)
    org_db = org.Create_organisation

    return jsonify("Success: Org Inserted")


@APP.route('/api/organisation/delete', methods=['DELETE'])
@cross_origin()
def org_delete():
    content = request.get_json()
    org_name = content.get('organisation')

    org = user_data.Organisation_db(org_name)
    org_db = org.Delete_organisation
    return jsonify("successfully deleted")


@APP.route('/api/login', methods=['POST'])
@cross_origin()
def login_user():
    if request.method == 'POST':
        return login_byonic_users()


@APP.route('/api/company/<identifier>', methods=['GET'])
@cross_origin()
def viewcompany(identifier):
    companyinfo = companyCard.CompanyCard(identifier)
    company = companyinfo.viewCompany()
    return jsonify(company)


@APP.route('/api/passwordreset/<identifier>', methods=['PUT'])
@cross_origin()
def Reset_Password(identifier):
    if request.method == 'PUT':
        id = identifier
        content = request.get_json()
        resetid = content.get('resetid')
        usernewpassword = content.get('newpassword')
        reset = resetPassword(id, usernewpassword, resetid).NEW_PASSWORD()
        if reset:
            return jsonify('Password Reset Successful')


@APP.route('/api/byonic_user', methods=['GET', 'POST'])
@cross_origin()
def get_user():
    if request.method == 'GET':
        return get_byonic_users()

    if request.method == "POST":
        return create_byonic_user()


@APP.route('/api/byonic_user/<identifier>', methods=['GET', 'PUT', 'DELETE',])
def get_user_by_id(identifier):
    if request.method == 'GET':
        return get_byonic_users(identifier)

    elif request.method == 'PUT':
        return update_byonic_user(identifier)

    elif request.method == 'DELETE':
        return delete_byonic_user(identifier)

@APP.route('/api/byonic_user/role', methods=['GET', 'POST'])
@cross_origin()
def get_user_role():
    if request.method == 'GET':
        return get_user_roles()

    if request.method == "POST":
        return create_user_role()


@APP.route('/api/byonic_user/role/<identifier>', methods=['GET', 'PUT', 'DELETE'])
def get_user_role_by_id(identifier):
    if request.method == 'GET':
        return get_user_role(identifier)

    elif request.method == 'PUT':
        return update_user_role(identifier)

    elif request.method == 'DELETE':
        return delete_user_role(identifier)


@APP.route('/api/companies/<identifier>', methods=['GET', 'PUT', 'DELETE'])
def get_company_by_id(identifier):
    if request.method == 'GET':
        return get_domain_intent_score(identifier)


if __name__ == "__main__":
    PORT = config.get_aibm_api_port()
    APP.config['SECRET_KEY'], APP.config['EXPIRY_TIME'] = get_login_config()
    APP.run(host='0.0.0.0', port=PORT, threaded=True)
