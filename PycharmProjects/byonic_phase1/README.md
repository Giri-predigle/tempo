# predigle_byonic_api
This repo contains the codes for byonic rest api services

Dataset
---
Contains intent-signal csv file

config
----
Contains Json files where I mentioned the filepath for storage, reading etc..

logs
----
Contains the logger info

scripts
-----
code files helps while implementing rest-api services
It list 3 codes:
* response.py -> code file deals with the ilead database connection
* filter_model_list.py -> code file that deals with filter concept
* ilead_model.py -> code file that helps to show all domains in intent-signal

byonic_api_integration.py
---------
* A code file that deals with all the rest api services
* When you run this code you will get all the rest full api urls

The urls are:

Post Method
* http://localhost/api/intent/signal [filter api url]

Get Method
* http://localhost/api/intent/signal-get [intent signal full domain request]
* http://localhost/api/sidenav [sidenav full response]
* http://localhost/api/sidenav/industries [sidenav industry response]
* http://localhost/api/sidenav/countries [sidenav country response]
* http://localhost/api/sidenav/job_levels [sidenav job level response]
* http://localhost/api/sidenav/emp_sizes [sidenav employee size response]
* http://localhost/api/sidenav/topics [sidenav topic response]

requirement.txt
-------
Contains the python packages need to be installed for running this code

Storage
------

The url response json json files are stored in the following folders

                  URL                                   Storage Folder
* http://localhost/api/intent/signal &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  test_API_storage     
* http://localhost/api/intent/signal-get &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   test_API_storage
* http://localhost/api/sidenav           &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;           test_storage
* http://localhost/api/sidenav/industries  &nbsp;&nbsp;&nbsp;           test_industry
* http://localhost/api/sidenav/countries   &nbsp;&nbsp;&nbsp;           test_country
* http://localhost/api/sidenav/topics      &nbsp;&nbsp;&nbsp;           test_topic
* http://localhost/api/sidenav/job_levels  &nbsp;&nbsp;&nbsp;           test_job
* http://localhost/api/sidenav/emp_sizes   &nbsp;&nbsp;&nbsp;           test_employee

