import json
import datetime
import time
import pandas as pd
import os
import glob


class MLModel:

    def __init__(self):
        """
        initialize variables
        """
        self.cwd = os.getcwd()
        config_file = self.cwd + '/byonic_data_access/config_file.json'

        with open(config_file, 'r') as file:
            api_config = json.load(file)
            api_config_details = api_config['Path']
        file.close()

        self.config_api_path = api_config_details['CONFIG_API_Path']
        self.data_csv_path = api_config_details['DATA_CSV_PATH']
        self.api_json_path = api_config_details['API_JSON_PATH']

        if 'DebugMode' in api_config and api_config['DebugMode'].upper() == "TRUE":
            self.debug_mode = True
        else:
            self.debug_mode = False

    def model(self):
        """
        :return:
        """

        file_path = self.cwd + '/' + self.data_csv_path
        globed = glob.glob(file_path + '/' + '*.csv')
        data = pd.read_csv(globed[0])
        data['sno'] = data.index
        col_name = "sno"
        first_col = data.pop(col_name)
        data.insert(0, col_name, first_col)
        data.columns = data.columns.str.lower()
        complete_filter_data = data.to_dict('records')
        resp_data = {'Domains': complete_filter_data}
        if self.debug_mode:
            get_time = time.strftime("%Y%m%d-%H%M%S")
            # output_json_list = "response-" + get_time + "-byonic_data_access.json"
            # # print('resp', output_json_list)
            # out_json_name = self.cwd + "/{}/{}".format(self.api_json_path, output_json_list)
            # with open(out_json_name, 'w', encoding='utf8') as jsl:
            #     json.dump(resp_data, jsl, indent=4)
            #     jsl.close()
            # # print('output', jsl)
        return resp_data
