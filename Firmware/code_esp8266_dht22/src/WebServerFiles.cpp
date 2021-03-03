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

            <label>SSID</label>
            <input id="input_network_ap_ssid" type="text" placeholder="SSID" name="network_ap_ssid" width="50%" border-radius="2px">
            <br>
            <label>Password</label>
            <input id="input_network_ap_pass" type="password" placeholder="Password" name="network_ap_pass" width="50%">
            <br><br>
            <select id="selector_network_connection_type">
                <option value="WPA2">WPA2</option>
                <option value="WPA2E">WPA2-Enterprise</option>
            </select>
            <br>
            <div id="div_wpa2e_parameters" style="display: none; border:1px solid gray; border-radius:5px; margin: 10px;">
                WPA2-Enterprise 
                <input style="display: none;" id="input_network_connection_type" type="text" placeholder="" name="network_connection_type">
                <br>
                <label>Usuario</label>
                <input id="input_network_wpa2_enterprise_user" type="text" placeholder="Usuario" name="network_wpa2_enterprise_user" width="50%">
                <br>
                <label>Identidad</label>
                <input id="input_network_wpa2_enterprise_identity" type="text" placeholder="Identidad" name="network_wpa2_enterprise_identity" width="50%">
                <br>
                <label>Contrase&ntilde;a</label>
                <input id="input_network_wpa2_enterprise_pass" type="password" placeholder="Contrase&ntilde;a" name="network_wpa2_enterprise_pass" width="50%">
                <br>
            </div>
            
            <br>
            <label>Local IP</label>
            <input id="input_client_static_ip" type="text" placeholder="Local IP" name="client_static_ip" width="50%">
            <br>
            <label>Gateway</label>
            <input id="input_client_gateway_ip" type="text" placeholder="Gateway IP" name="client_gateway_ip" width="50%">
            <br>
            <label>Subnet</label>
            <input id="input_client_subnet_mask" type="text" placeholder="Subnet mask" name="client_subnet_mask" width="50%">
            <br>
            <label>DNS1</label>
            <input id="input_client_dns1_ip" type="text" placeholder="DNS1 IP" name="client_dns1_ip" width="50%">
            <br>
            <label>DNS2</label>
            <input id="input_client_dns2_ip" type="text" placeholder="DNS2 IP" name="client_dns2_ip" width="50%">
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
            <label>Host/IP</label>
            <input id="input_server_ip" type="text" placeholder="IPv4" name="server_ip" width="50%" border-radius="2px">
            <br>
            <label>Puerto</label>
            <input id="input_server_port" type="text" placeholder="Puerto" name="server_port" width="50%">
            <br>
            <label>Ruta mediciones</label>
            <input id="input_send_measurements_path" type="text" placeholder="Ruta datos" name="send_measurements_path" width="50%">
            <br>
            <label>Ruta sincronizaci&oacute;n<nav></nav></label>
            <input id="input_get_time_path" type="text" placeholder="Ruta sync." name="get_time_path" width="50%">
            <br>
            <input class="button" type="submit" value="Modificar">
            
        </form>
    </div>
    <br><br>
    <div id="box_memory" class="box center">
        <b>Memorias</b>
        <br><br>
        <button class="button" onclick="formatRam()">Formatear RAM</button>
        <button class="button" onclick="formatFlash()">Formatear FLASH</button>
        <button class="button" onclick="resetSentPointer()">Resetear puntero send</button>
    </div>
    <br><br>

    <div id="input_device_config " class="box center">
        <b>Dispositivo</b>
        <p style="font-size: 12px;">Par&aacute;metros</p>
        <br>
        
        <div class="device_config">
            <label>Timeout conexi&oacute;n (ms)</label>
            <input type="text" id="input_network_connection_timeout">
        </div>
        <br>
        <div class="device_config">
            <label>Reintentos de conexi&oacute;n</label>
            <input type="text" id="input_server_connection_retry">
        </div>
        <br>
        <div class="device_config">
            <label>ID Dispositivo</label>
            <input type="text" id="input_id_transceiver">
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
            <label>Tiempo muestreo (seg)</label>
            <input type="text" id="input_sample_time">
        </div>
        <br>
        <div class="device_config">
            <label>Hora conexi&oacute;n (UTC)</label>
            <input type="text" id="input_connection_time" placeholder="Hs">
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
        
        //Request parameters saved into device:
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                console.log("responseText:",this.responseText);
                parameters = JSON.parse(JSON.parse(this.responseText));

                console.log("parameters:", parameters);
                
                document.querySelector("#input_network_ap_ssid").value = parameters["network_ap_ssid"];
                document.querySelector("#input_network_ap_pass").value = parameters["network_ap_pass"];
                
                document.querySelector("#input_network_connection_type").value = parameters["network_connection_type"];
                document.querySelector("#input_network_wpa2_enterprise_user").value = parameters["network_wpa2_enterprise_user"];
                document.querySelector("#input_network_wpa2_enterprise_identity").value = parameters["network_wpa2_enterprise_identity"];
                document.querySelector("#input_network_wpa2_enterprise_pass").value = parameters["network_wpa2_enterprise_pass"];
                
                document.querySelector("#input_client_static_ip").value = parameters["client_static_ip"];
                document.querySelector("#input_client_gateway_ip").value = parameters["client_gateway_ip"];
                document.querySelector("#input_client_subnet_mask").value = parameters["client_subnet_mask"];
                document.querySelector("#input_client_dns1_ip").value = parameters["client_dns1_ip"];
                document.querySelector("#input_client_dns2_ip").value = parameters["client_dns2_ip"];

                document.querySelector("#input_server_ip").value = parameters["server_ip"];
                document.querySelector("#input_server_port").value = parameters["server_port"];
                document.querySelector("#input_send_measurements_path").value = parameters["send_measurements_path"];
                document.querySelector("#input_get_time_path").value = parameters["get_time_path"];
                document.querySelector("#input_network_connection_timeout").value = parameters["network_connection_timeout"];
                document.querySelector("#input_server_connection_retry").value = parameters["server_connection_retry"];
                document.querySelector("#input_id_transceiver").value = parameters["id_transceiver"];
                document.querySelector("#input_id_sensor_a").value = parameters["id_sensor_a"];
                document.querySelector("#input_id_sensor_b").value = parameters["id_sensor_b"];
                document.querySelector("#input_id_sensor_c").value = parameters["id_sensor_c"];
                document.querySelector("#input_id_sensor_d").value = parameters["id_sensor_d"];
                document.querySelector("#input_sample_time").value = parameters["sample_time"];
                document.querySelector("#input_connection_time").value = parameters["connection_time"];

                // console.log("JSON parsed:",JSON.parse(this.responseText));
               
                // handle JSON from ESP8266...
            }
        };

        xhttp.open("POST", "get_parameters", true);     //Request to "/get_parameters" URL.
        xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
        xhttp.send("command=get_parameters");   //Send request
        

    });

    document.querySelector("#selector_network_connection_type").addEventListener('click', function(){

        // console.log('Auth type clicked:', this.options[this.options.selectedIndex].value);

        if(this.options[this.options.selectedIndex].value == 'WPA2E'){
            document.querySelector("#div_wpa2e_parameters").style.display='block';
        }else{
            document.querySelector("#div_wpa2e_parameters").style.display='none';
        }
        document.querySelector("#input_network_connection_type").value=this.value;


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
        if (confirm("¿Apagar dispositivo?\nDesconecte el cable USB.")) {
            if (!alert("Desconecte el cable USB.")) {            
                console.log("Device will turn off...");
                var xhttp = new XMLHttpRequest();
                xhttp.open("POST", "device_turn_off", true);
                xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
                xhttp.send("command=turn_off_confirm");
            }
            else console.log("Turn off cancelled.");  //Cancelled
        }
        else {
            console.log("Turn off cancelled.");  //Cancelled
        }
    }

    function deviceConfig() {
        
        //Update values to variable.
        parameters["network_ap_ssid"] = document.querySelector("#input_network_ap_ssid").value;
        parameters["network_ap_pass"] = document.querySelector("#input_network_ap_pass").value;

        parameters["network_connection_type"] = document.querySelector("#input_network_connection_type").value;
        parameters["network_wpa2_enterprise_user"] = document.querySelector("#input_network_wpa2_enterprise_user").value;
        parameters["network_wpa2_enterprise_identity"] = document.querySelector("#input_network_wpa2_enterprise_identity").value;
        parameters["network_wpa2_enterprise_pass"] = document.querySelector("#input_network_wpa2_enterprise_pass").value;

        parameters["client_static_ip"] = document.querySelector("#input_client_static_ip").value;
        parameters["client_gateway_ip"] = document.querySelector("#input_client_gateway_ip").value;
        parameters["client_subnet_mask"] = document.querySelector("#input_client_subnet_mask").value;
        parameters["client_dns1_ip"] = document.querySelector("#input_client_dns1_ip").value;
        parameters["client_dns2_ip"] = document.querySelector("#input_client_dns2_ip").value;

        parameters["server_ip"] = document.querySelector("#input_server_ip").value;
        parameters["server_port"] = document.querySelector("#input_server_port").value;
        parameters["send_measurements_path"] = document.querySelector("#input_send_measurements_path").value;
        parameters["get_time_path"] = document.querySelector("#input_get_time_path").value;
        parameters["network_connection_timeout"] = document.querySelector("#input_network_connection_timeout").value;
        parameters["server_connection_retry"] = document.querySelector("#input_server_connection_retry").value;
        parameters["id_transceiver"] = document.querySelector("#input_id_transceiver").value;
        parameters["id_sensor_a"] = document.querySelector("#input_id_sensor_a").value;
        parameters["id_sensor_b"] = document.querySelector("#input_id_sensor_b").value;
        parameters["id_sensor_c"] = document.querySelector("#input_id_sensor_c").value;
        parameters["id_sensor_d"] = document.querySelector("#input_id_sensor_d").value;
        parameters["sample_time"] = document.querySelector("#input_sample_time").value;
        parameters["connection_time"] = document.querySelector("#input_connection_time").value;

        
        var xhttp = new XMLHttpRequest();
        xhttp.open("POST", "device_config", true);     //Request to "/get_parameters" URL.
        xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
        xhttp.send(
            `network_ap_ssid=${parameters["network_ap_ssid"]}&`+
            `network_ap_pass=${parameters["network_ap_pass"]}&`+
            
            `network_connection_type=${parameters["network_connection_type"]}&`+
            `network_wpa2_enterprise_user=${parameters["network_wpa2_enterprise_user"]}&`+
            `network_wpa2_enterprise_identity=${parameters["network_wpa2_enterprise_identity"]}&`+
            `network_wpa2_enterprise_pass=${parameters["network_wpa2_enterprise_pass"]}&`+

            `client_static_ip=${parameters["client_static_ip"]}&`+
            `client_gateway_ip=${parameters["client_gateway_ip"]}&`+
            `client_subnet_mask=${parameters["client_subnet_mask"]}&`+
            `client_dns1_ip=${parameters["client_dns1_ip"]}&`+
            `client_dns2_ip=${parameters["client_dns2_ip"]}&`+
            
            `server_ip=${parameters["server_ip"]}&`+
            `server_port=${parameters["server_port"]}&`+
            `send_measurements_path=${parameters["send_measurements_path"]}&`+
            `get_time_path=${parameters["get_time_path"]}&`+
            `connection_time=${parameters["connection_time"]}&`+
            `network_connection_timeout=${parameters["network_connection_timeout"]}&`+
            `server_connection_retry=${parameters["server_connection_retry"]}&`+
            `id_transceiver=${parameters["id_transceiver"]}&`+
            `id_sensor_a=${parameters["id_sensor_a"]}&`+
            `id_sensor_b=${parameters["id_sensor_b"]}&`+
            `id_sensor_c=${parameters["id_sensor_c"]}&`+
            `id_sensor_d=${parameters["id_sensor_d"]}&`+
            `sample_time=${parameters["sample_time"]}`
        );   //Send request
    }       
</script>
</html>
)rawliteral";

