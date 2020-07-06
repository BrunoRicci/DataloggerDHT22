#include <avr/pgmspace.h>
#include <ESP8266WebServer.h>




const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1" charset="UTF-8">
  <style>
    html {
     font-family:Arial, Helvetica, sans-serif;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
     min-width: 240px;
    }
    h2 { font-size: 2.0rem; }
    .units { font-size: 1.2rem; }
    .dht-labels{
        font-size: 1.5rem;
        vertical-align:middle;
        padding-bottom: 15px;
    }
    .box{
        background-color: #e7e7e7;
        width: 85%;
        border: 2px solid #3a67fc ;
        border-radius: 5px;
        padding: 20px;
        text-align: center;
    }
    .center {
    margin: auto;
    padding: 10px;
    }
    .button{
        background-color: rgb(201, 201, 201);
        border: 2px solid rgb(65, 65, 65);
        border-radius: 7px;
        padding: 5px 14px;
        margin: 10px;
        width: 120px;
    }
    input{
        padding: 5px;
        margin: 2px;
        border-radius: 10px;
        border: 1px solid black;
        width: 60%;
        min-width: 30px;
        text-align: center;
    }
    .device_config{
        display: inline;
        /* width: 40%; */
    }
  </style>
</head>
<body>
    <h2>Configuraci&oacute;n del datalogger</h2>
  
    <div id="box_network_config" class="box center">
        <b>Red</b>
        <br>
        <br>
        <form method="POST" action="change_network_config">
            <input type="text" placeholder="SSID" name="new_ssid" width="50%" border-radius="2px">
            <br>
            <input type="text" placeholder="Password" name="new_password" width="50%">
            <br>
            <input class="button" type="submit" value="Modificar">
            
        </form>
        
    </div>
    <br><br>
    <div id="box_server_config" class="box center">
        <b>Servidor</b>
        <br>
        <br>
        <form method="POST" action="change_server_config" margin="10px">
            <input type="text" placeholder="IPv4" name="ip" width="50%" border-radius="2px">
            <br>
            <input type="text" placeholder="Puerto" name="port" width="50%">
            <br>
            <input class="button" type="submit" value="Modificar">
            
        </form>
    </div>
    <br><br>
    <div id="box_memory" class="box center">
        <b>Memorias</b>
        <br>
        <br>
        <button class="button" onclick="formatRam()">Formatear RAM</button>
        <button class="button" onclick="formatFlash()">Formatear FLASH</button>
        <button class="button" onclick="resetSentPointer()">Resetear puntero send</button>
    </div>
    <br><br>

    <div id="device_config " class="box center">
        <b>Dispositivo</b>
        <p style="font-size: 12px;">Par&aacute;metros</p>
        <br>
        
        <div class="device_config">
            <label>Timeout conexi&oacute;n</label>
            <input type="text" id="input_connection_timeout">
        </div>
        <br>
        <div class="device_config">
            <label>Reintentos de conexi&oacute;n</label>
            <input type="text" id="input_connection_retries">
        </div>
        <br>
        <div class="device_config">
            <label>ID Dispositivo</label>
            <input type="text" id="input_transceiver_id">
        </div>
        <br>
        <div class="device_config">
            <label>ID Sensor A</label>
            <input type="text" id="input_id_sensor_a">
        </div>
        <br>
        <div class="device_config">
            <label>ID Sensor B</label>
            <input type="text" id="input_id_sensor_b">
        </div>
        <br>
        <div class="device_config">
            <label>ID Sensor C</label>
            <input type="text" id="input_id_sensor_c">
        </div>
        <br>
        <div class="device_config">
            <label>ID Sensor D</label>
            <input type="text" id="input_id_sensor_d">
        </div>
        <br>
        <div class="device_config">
            <label>Tiempo muestreo</label>
            <input type="text" id="input_sample_time">
        </div>

        

        <br>
        <button class="button" onclick="deviceConfig()">Cambiar par&aacute;metros</button>
        <br>
        <button class="button" style="background-color: #ff9898;" onclick="turnOffDevice()">Apagar</button>
    </div>

</body>
<!-- ---------------------------------------------------- -->
<script>
    var parameters = {};    //Global

    window.addEventListener('load', function(){
        // var transceiver_id = document.querySelector("#input_transceiver_id");

        // console.log("tid value:",transceiver_id.value);
        // console.log("tid length:",transceiver_id.value.length);

        //Request parameters saved into device:
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                console.log("responseText:",this.responseText);
                parameters = JSON.parse(JSON.parse(this.responseText));
                console.log("parameters:", parameters);
                
                console.log(parameters["server_ap_ssid"]);
                console.log(parameters["server_ap_pass"]);
                console.log(parameters["server_ip"]);
                console.log(parameters["server_port"]);
                console.log(parameters["network_connection_timeout"]);
                console.log(parameters["server_connection_retry"]);
                console.log(parameters["id_transceiver"]);
                console.log(parameters["id_sensor_a"]);
                console.log(parameters["id_sensor_b"]);
                console.log(parameters["id_sensor_c"]);
                console.log(parameters["id_sensor_d"]);
                
                // console.log("JSON parsed:",JSON.parse(this.responseText));
                debugger;
                //responseText should be separated into all the different values. See how to
                // handle JSON from ESP8266...
            }
        };

        xhttp.open("POST", "get_parameters", true);     //Request to "/get_parameters" URL.
        xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
        xhttp.send("command=get_parameters");   //Send request
        

    });


    function formatRam(){
        if (confirm("¿Confirma que desea formatear la RAM?")) {
            console.log("RAM memory will format...");
            var xhttp = new XMLHttpRequest();
            xhttp.open("POST", "format_ram", true);
            xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
            xhttp.send("command=format_confirm");          //Actual data to be sent
        } 
        else {
            console.log("RAM memory format cancelled.");//Cancelled
        }
    }

    function formatFlash(){
        if (confirm("¿Confirma que desea formatear la FLASH?")) {
            console.log("FLASH memory will format...");
            var xhttp = new XMLHttpRequest();
            xhttp.open("POST", "format_flash", true);
            xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
            xhttp.send("command=format_confirm");
        }
        else {
            console.log("FLASH memory format cancelled.");  //Cancelled
        }
    }

    function resetSentPointer(){
        if (confirm("¿Confirma que desea resetear el puntero?")) {
            console.log("Resetting archive_sent_pointer...");
            var xhttp = new XMLHttpRequest();
            xhttp.open("POST", "reset_archive_sent_pointer", true);
            xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
            xhttp.send("command=format_confirm");
        }
        else {
            console.log("archive_send_pointer reset cancelled.");  //Cancelled
        }
    }

    function turnOffDevice(){
        if (confirm("¿Apagar dispositivo?")) {
            console.log("Device will turn off...");
            var xhttp = new XMLHttpRequest();
            xhttp.open("POST", "device_turn_off", true);
            xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
            xhttp.send("command=turn_off_confirm");
        }
        else {
            console.log("Turn off cancelled.");  //Cancelled
        }
    }

    function deviceConfig() {
      

    }

       
    /* 
    strcpy(config_globals.server_ap_ssid,"Fibertel WiFi866 2.4GHz");
    strcpy(config_globals.server_ap_pass,"01416592736");
    strcpy(config_globals.server_ip, "192.168.0.172");
    strcpy(config_globals.wifi_security_type,"WPA2");

    config_globals.server_port=8080;
    config_globals.connection_retry=2;
    config_globals.connection_timeout=20000;
    config_globals.server_connection_timeout=2000;
    config_globals.response_timeout=5000;
    config_globals.id_transceiver=1;
    config_globals.id_sensor_1=1;
    config_globals.id_sensor_2=2;
    config_globals.id_sensor_3=3;
    config_globals.id_sensor_4=4;
    config_globals.sample_time=3600;
    config_globals.connection_time[0]=3;  //6:30 a.m. UTC
    config_globals.connection_time[0]=30;
     */

</script>
</html>
)rawliteral";

