


const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {
     font-family: Arial;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
    }
    h2 { font-size: 3.0rem; }
    p { font-size: 3.0rem; }
    .units { font-size: 1.2rem; }
    .dht-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
    }
    .box{
    background-color: #d3d3d3;
    width: 300px;
    border: 2px solid #3a67fc ;
    border-radius: 5px;
    padding: 20px;
    text-align: center;
    }
    .center {
    margin: auto;
    padding: 10px;
    }
  </style>
</head>
<body>
    <h2>Configuración del datalogger</h2>
  
    <div id="box_network_config" class="box center">
        <b>Red</b>
        <br>
        <br>
        <form method="POST" action="change_network_config">
            <input type="text" placeholder="SSID" name="new_ssid" width="50%" border-radius="2px">
            <br>
            <input type="text" placeholder="Password" name="new_password" width="50%">
            <br>
            <input type="submit" value="Modificar" width="30%">
            
        </form>
        
    </div>
    <br><br>
    <div id="box_server_config" class="box center">
        <b>Servidor</b>
        <br>
        <br>
        <form method="POST" action="change_server_config">
            <input type="text" placeholder="IPv4" name="IP" width="50%" border-radius="2px">
            <br>
            <input type="text" placeholder="Puerto" name="port" width="50%">
            <br>
            <input type="submit" value="Modificar" width="30%">
            
        </form>
    </div>
    <br><br>
    <div id="box_memory" class="box center">
        <b>Memorias</b>
        <br>
        <br>

        <button onclick="formatRam()">Formatear RAM</button>
        <button onclick="formatFlash()">Formatear FLASH</button>


    
    </div>

</body>
<!-- ---------------------------------------------------- -->
<script>
    function formatRam(){
        if (confirm("¿Confirma que desea formatear la RAM?")) {
            console.log("RAM memory will format...");
            var xhttp = new XMLHttpRequest();
            xhttp.open("POST", "format_ram", true);
            xhttp.send();
        } 
        else {
            console.log("RAM memory format cancelled.");//Cancelled
        }
    }

    function formatFlash(){
        if (confirm("¿Confirma que desea formatear la FLASH?")) {
            console.log("FLASH memory will format...");
            var xhttp = new XMLHttpRequest();
            xhttp.open("POST", "format_ram", true);
            xhttp.send();
        } 
        else {
            console.log("FLASH memory format cancelled.");//Cancelled
        }
    }
</script>
</html>)rawliteral";