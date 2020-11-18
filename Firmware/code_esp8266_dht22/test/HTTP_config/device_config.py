import requests
import json



devices_data={
    "1":{
        "network_ap_ssid":"Loggers",
        "network_ap_pass":"crnvrs2020!",
        "client_static_ip":"0.0.0.0",
        "client_gateway_ip":"0.0.0.0",
        "client_subnet_mask":"255.255.0.0",
        "client_dns1_ip":"200.49.130.47",
        "client_dns2_ip":"200.42.4.204",
        "server_ip":"ifeva.edu.ar",
        "server_port":80,
        "send_measurements_path":"/sensores/test.php",
        "get_time_path":"/sensores/gettime.php",
        "network_connection_timeout":15000,
        "server_connection_retry":5,
        "id_transceiver":1,
        "id_sensor_a":1,
        "id_sensor_b":2,
        "id_sensor_c":3,
        "id_sensor_d":4,
        "sample_time":3600,
        "connection_time":9
    },
    "2":{
        "network_ap_ssid":"Loggers",
        "network_ap_pass":"crnvrs2020!",
        "client_static_ip":"0.0.0.0",
        "client_gateway_ip":"0.0.0.0",
        "client_subnet_mask":"255.255.0.0",
        "client_dns1_ip":"200.49.130.47",
        "client_dns2_ip":"200.42.4.204",
        "server_ip":"ifeva.edu.ar",
        "server_port":80,
        "send_measurements_path":"/sensores/test.php",
        "get_time_path":"/sensores/gettime.php",
        "network_connection_timeout":15000,
        "server_connection_retry":5,
        "id_transceiver":2,
        "id_sensor_a":5,
        "id_sensor_b":6,
        "id_sensor_c":7,
        "id_sensor_d":8,
        "sample_time":3600,
        "connection_time":9
    },
    "3":{
        "network_ap_ssid":"Loggers",
        "network_ap_pass":"crnvrs2020!",
        "client_static_ip":"0.0.0.0",
        "client_gateway_ip":"0.0.0.0",
        "client_subnet_mask":"255.255.0.0",
        "client_dns1_ip":"200.49.130.47",
        "client_dns2_ip":"200.42.4.204",
        "server_ip":"ifeva.edu.ar",
        "server_port":80,
        "send_measurements_path":"/sensores/test.php",
        "get_time_path":"/sensores/gettime.php",
        "network_connection_timeout":15000,
        "server_connection_retry":5,
        "id_transceiver":3,
        "id_sensor_a":9,
        "id_sensor_b":10,
        "id_sensor_c":11,
        "id_sensor_d":12,
        "sample_time":3600,
        "connection_time":9
    },
    "4":{
        "network_ap_ssid":"Loggers",
        "network_ap_pass":"crnvrs2020!",
        "client_static_ip":"0.0.0.0",
        "client_gateway_ip":"0.0.0.0",
        "client_subnet_mask":"255.255.0.0",
        "client_dns1_ip":"200.49.130.47",
        "client_dns2_ip":"200.42.4.204",
        "server_ip":"ifeva.edu.ar",
        "server_port":80,
        "send_measurements_path":"/sensores/test.php",
        "get_time_path":"/sensores/gettime.php",
        "network_connection_timeout":15000,
        "server_connection_retry":5,
        "id_transceiver":4,
        "id_sensor_a":13,
        "id_sensor_b":14,
        "id_sensor_c":15,
        "id_sensor_d":16,
        "sample_time":3600,
        "connection_time":9
    },
    "5":{
        "network_ap_ssid":"Loggers",
        "network_ap_pass":"crnvrs2020!",
        "client_static_ip":"0.0.0.0",
        "client_gateway_ip":"0.0.0.0",
        "client_subnet_mask":"255.255.0.0",
        "client_dns1_ip":"200.49.130.47",
        "client_dns2_ip":"200.42.4.204",
        "server_ip":"ifeva.edu.ar",
        "server_port":80,
        "send_measurements_path":"/sensores/test.php",
        "get_time_path":"/sensores/gettime.php",
        "network_connection_timeout":15000,
        "server_connection_retry":5,
        "id_transceiver":5,
        "id_sensor_a":17,
        "id_sensor_b":18,
        "id_sensor_c":19,
        "id_sensor_d":20,
        "sample_time":3600,
        "connection_time":9
    },
    "6":{
        "network_ap_ssid":"Fibertel WiFi866 2.4GHz",
        "network_ap_pass":"01416592736",
        "client_static_ip":"192.168.0.206",
        "client_gateway_ip":"192.168.0.1",
        "client_subnet_mask":"255.255.255.0",
        "client_dns1_ip":"200.49.130.47",
        "client_dns2_ip":"200.42.4.204",
        "server_ip":"ifeva.edu.ar",
        "server_port":80,
        "send_measurements_path":"/sensores/test.php",
        "get_time_path":"/sensores/gettime.php",
        "network_connection_timeout":15000,
        "server_connection_retry":5,
        "id_transceiver":6,
        "id_sensor_a":21,
        "id_sensor_b":22,
        "id_sensor_c":23,
        "id_sensor_d":24,
        "sample_time":3600,
        "connection_time":9
    }
}

r = requests.request('post','http://192.168.0.170:8080/get_parameters', headers={'Accept':'text'}).json()
device_id = str(json.loads(r)['id_transceiver'])
print('device id:'+device_id)

device_id='6'   #FORCE DEVICE ID
device_data = devices_data[device_id]
x = device_data

# d = json.dumps(device_data, separators=(',',':'))   

# Convert to urlencoded format
values=''
for i in x.keys():
    values += i + '=' + str(x[i]) + '&'
values = values[0:-1]   # Removes last "&"

device_parameters = values
print(device_parameters)



send = requests.request('post','http://192.168.0.170:8080/change_network_config',
                        headers={'Content-type':'application/x-www-form-urlencoded'},
                        data=device_parameters
                        )
print(send)

send = requests.request('post','http://192.168.0.170:8080/change_server_config',            #values names are not in this format
                        headers={'Content-type':'application/x-www-form-urlencoded'},
                        data=device_parameters
                        )
print(send)

send = requests.request('post','http://192.168.0.170:8080/device_config',               #values names are not in this format
                        headers={'Content-type':'application/x-www-form-urlencoded'},
                        data=device_parameters
                        )
print(send)




# for i in j.keys():
#     print(i+": "+str(j[i]) + '  /type:'+str(type(j[i])))



