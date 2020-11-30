import os
import sys
from bottle import run, template, route, request, static_file, Response, app, TEMPLATE_PATH
from datahandle import dataeng
import time
import datetime
TEMPLATE_PATH.insert(0, 'C:/Users/BRUNO - PC/REPOS_GIT/DataloggerDHT22/Tests web server/Python/webserver')

NAME_DB = 'fauba_dataloggers.db'
DIRECTORY_DB = "C:/Users/BRUNO - PC/REPOS_GIT/DataloggerDHT22/Tests web server/Python/webserver"
MEASUREMENTS_TABLE_NAME = 'MEASUREMENTS'
DEVICES_STATE_TABLE_NAME = 'DEVICES_STATUS'
DB_COLUMN_NAME_ID_TRANSCEIVER = 'ID_TRANSCEIVER'
DB_COLUMN_NAME_BATTERY_LEVEL = 'BATTERY_LEVEL'
DB_COLUMN_NAME_ID_SENSOR = 'ID_SENSOR'
DB_COLUMN_NAME_TIMESTAMP = 'TIMESTAMP'
DB_COLUMN_NAME_DATE_TIME = 'DATE_TIME'
DB_COLUMN_NAME_TEMPERATURE = 'TEMPERATURE'
DB_COLUMN_NAME_HUMIDITY = 'HUMIDITY'


localdir = str(sys.path[0])
def initDB(dir, nameDB):   
    print('DB directory to open: ' + str(dir + '\\' + nameDB))   #debug
    db = dataeng.DataEngine( dir + '\\' + nameDB, debugmode=False)  #Create DataEngine object with the database directory.  Will create dir if it doesn't exist.
    # db = dataeng.sqlite3.connect(dir + '\\' + nameDB)  # Access a database. Create it if doesn't exist.
    # print('db = ' + str(db))
    return db

def listTables (bdd):
    bdd.execute('SELECT name FROM sqlite_master')   #Gets every table name from the DB.
    return dataeng.dbResultToList(bdd.fetchall())  

def startDB(db):
    tables_list = dataeng.getDbTables(db)
    if MEASUREMENTS_TABLE_NAME not in tables_list:        #if the table doesn't exist, creates it.
        db.executeCommand(""" CREATE TABLE {} (
                                TIMESTAMP       INTEGER     
                            ,   DATE_TIME       INTEGER     
                            ,   ID_SENSOR       INTEGER	
                            ,   TEMPERATURE     REAL
                            ,   HUMIDITY        REAL
                            , 	CONSTRAINT COMPOSITE_PK PRIMARY KEY (ID_SENSOR, TIMESTAMP)
                            );
                            """.format(MEASUREMENTS_TABLE_NAME))
        print('Table {} created.'.format(MEASUREMENTS_TABLE_NAME))  #Print the created table name.
    
    if DEVICES_STATE_TABLE_NAME not in tables_list:        #if the table doesn't exist, creates it.
        db.executeCommand(""" CREATE TABLE {} (
                                TIMESTAMP       TEXT
                            ,   ID_TRANSCEIVER  INTEGER
                            ,   BATTERY_LEVEL   INTEGER        
                            , 	CONSTRAINT COMPOSITE_PK PRIMARY KEY (ID_TRANSCEIVER, TIMESTAMP)
                            );
                            """.format(DEVICES_STATE_TABLE_NAME))
        print('Table {} created.'.format(DEVICES_STATE_TABLE_NAME))  #Print the created table name.

    print('DB tables:' + str(listTables))       #List all tables in the db.

def handleReceivedData(dict_data):
    #manages data received from the node. 
    data_valid = True   #Validate data integrity.
    
    columns=[
            DB_COLUMN_NAME_TIMESTAMP,
            DB_COLUMN_NAME_DATE_TIME,
            DB_COLUMN_NAME_ID_SENSOR,
            DB_COLUMN_NAME_TEMPERATURE,
            DB_COLUMN_NAME_HUMIDITY
            ]


    if data_valid == True:
        startDB(actual_db)  #start database tables.
        time_modifier = 0
        print('Saving data...')
       
        while time_modifier < 10:   #max 10 seconds.
            values = []
            for i in range(0,len(dict_data["timestamp"])):       #for each measurement value...
                measurement = [
                                dict_data["timestamp"][i] + time_modifier,
                                str(datetime.datetime.fromtimestamp(dict_data["timestamp"][i]+time_modifier)),    #Convert epoch to current time (UTC -3:00)
                                dict_data["id_sensor"][i],
                                dict_data["temperature"][i],
                                dict_data["humidity"][i]
                            ]
                values.append(measurement)  #Append list of values to the values-to-store list.
                
            # print('Data to be saved:' + str(values)) 

            try:
                actual_db.writeData(MEASUREMENTS_TABLE_NAME, columns, values)   #Save to database.
            except IndexError:  
                print("\nException.\n")
                print(" - - - - - Timestamp: "+str(dict_data["timestamp"]))
                time_modifier = time_modifier + 1
                print("Time modifier: "+str(time_modifier))
                #print( "Has duplicates: "+str( len(set(dict_data["timestamp"])) ) )
            else:       #If data saved correcly...
                try:    #Try to save the status data. If timestamp (composite primary key) is repeated in the table, it will throw an exception.
                    actual_db.writeData(DEVICES_STATE_TABLE_NAME,                           #Logs devices state.
                                    ['TIMESTAMP','ID_TRANSCEIVER','BATTERY_LEVEL'], 
                                    [[dict_data["current_time"], dict_data["id_transceiver"], dict_data["battery_level"]]]
                                    )
                except:
                    pass

                return True     #Notices that data has been received correctly.

        return False
    else:
        return False    
    
def removeDuplicates(l):
    # Remove duplicates from a list.
    l = ["a", "b", "a", "c", "c"]
    l = list( dict.fromkeys(l) )
    return l

""" BUG: when saving device status timestamp, "timestamp" attribute may be duplicated due to very fast packet sending from client (faster than 1 sec.)
    then, if this happens, handle the primary key unique constraint failure NOT trying to write this value, as the timestamp is frequent enough for 
    this kind of data, which should actually be computed every time the device wakes up, not in every packet sent.
 """


actual_db = initDB(DIRECTORY_DB, NAME_DB)       #Creates object for database.

@route('/') 
def index():    #Returns home page
    return template('config_page.html')


@route('/test', method='GET')       
def test():             #Returns tha same that is sent via GET method.
    value = request.GET.get('value')
    
    print('Received value: {}'.format(value))
    return ('Received value: {valor}'.format(valor=value))

@route('/sensores/test.php', method='POST')
def sendMeasurements(): #change method name...
    dict_data = {"id_transceiver": 0,"battery_level":0,"timestamp": [],"id_sensor": [],"temperature":[],"humidity":[], "current_time":0}    


    print('Received request:'+str(request.POST.keys() ))
    # print('Received request values:'+str(request.POST.values() ))
    dict_data["id_transceiver"] = int(request.POST.get('id_transceiver'))
    dict_data["battery_level"] = int(request.POST.get('battery_level'))
    #Remove brackets from string request and splits between comma separated values.
    list_timestamp  =   (str(request.POST.get('timestamp')).translate({ord(i): None for i in '[]'})).split(sep=',')      
    list_id_sensor  =   (str(request.POST.get('id_sensor')).translate({ord(i): None for i in '[]'})).split(sep=',')      
    list_temperature=   (str(request.POST.get('temperature')).translate({ord(i): None for i in '[]'})).split(sep=',')      
    list_humidity   =   (str(request.POST.get('humidity')).translate({ord(i): None for i in '[]'})).split(sep=',')      
    
    for i in range(0,len(list_timestamp)):
        dict_data["timestamp"].append(int(list_timestamp[i]))
        dict_data["id_sensor"].append(int(list_id_sensor[i]))
        dict_data["temperature"].append(float(list_temperature[i])/10)
        dict_data["humidity"].append(int(list_humidity[i]))
    dict_data["current_time"] = int(time.time())    #Current request timestamp.
    # dict_data["id_transceiver"] = int( 1 )        #TO FORCE ID_TRANSCEIVER VALUE when isn't correctly configured.

    # Also convert from epoch time to SQL DATETIME format. 
    handleReceivedData(dict_data)
    
    print('Received values:'+str(dict_data))
    return 'Values received!' 

@route('/sensores/gettime.php')
def get_time():
    currenttime = int(time.time())    #Get current UNIX UTC time in seconds.
    result = "Current UNIX time (UTC):\n{}".format(str(currenttime))
    return (result)


@route('/change_network_config', method='POST')
def change_network_config():
    value = request.POST.keys()

    print('Received value: {}'.format(value))

    print('Valor recibido: {valor}'.format(valor=value))

@route('/change_server_config', method='POST')
def change_server_config():
    value = request.POST.keys()

    print('Received value: {}'.format(value))

    print('Valor recibido: {valor}'.format(valor=value))

@route('/format_ram', method='POST')
def format_ram():
    print('RAM Will be formatted.')

    
    return('RAM formatted.')

@route('/format_flash', method='POST')
def format_flash():
    print('FLASH Will be formatted.')

    return('FLASH formatted.')

try:
    # run(host='192.168.0.172', port=8080, reloader=True, debug=True)
    run(host='192.168.0.155', port=8080, reloader=True, debug=True)
    # run(host='192.168.1.101', port=8080, reloader=True, debug=True)
finally:    #when to finish program...
    actual_db.finishConn()  #closes database connection.
    print('Program finished.')

