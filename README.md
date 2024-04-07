
## Aplicacion distribuida de radar empleando ROS2 y RaspberryPi 5
<ul>
    <li>Fechas: Abril 2024</li>
    <li>Descripción: Se parte de una RaspberryPi 5, un servomotor y un sesor de ultrasonidos, ademas se necesita otro dispositivo compatible con ROS2 Humble. Empleando el framework ROS2 Humble se crea un nodo que contiene la "action interface" donde se define el tipo de mensaje que el el cliente debe enviar al servidor. Para la parte del servidor, se emplea la RaspberryPi 5 con el sistema operativo Raspberry Pi OS, basado en Debian por lo que no permite instalar ROS2, para solucionar esto se emplea una contenedor Docker con imagen de Ubuntu con ROS2 Humble instalado entre otras dependencias. Para poder controlar los sensores y actuadores se emplea la libreria Libgpiod. Una vez lanzado este paquete este espera a recibir la peticion de un cliente. En otro dispositivo con ROS2 Humble instalado y en red con la Raspberry Pi 5 se ubica el cliente, desde un "launch file" que ejecuta este nodo y con el que es posible modificar sus parametros. El cliente una vez enviado el mensaje empezara a recibir feedback constante con las posiciones actuales del servo y la distancias medidas por el sensor de ultrasonidos, con estos datos este genera una ventana donde se pintan las distintas señales que se han recibido durante todo el tiempo de escaneo, para esto ultimo se emplea OpenCV.  
   
![foto](https://github.com/asier-vega-gutierrez/ROS2_RPI5_Radar/blob/main/doc/EsquemaGeneral.png)

</li>
    <li>Resultados:</li>
</ul>

![foto](https://github.com/asier-vega-gutierrez/ROS2_RPI5_Radar/blob/main/doc/Demostracion.png)
