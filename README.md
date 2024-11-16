
## Aplicacion distribuida de radar empleando ROS2 y RaspberryPi 5
### English:
<ul>
    <li>Date: April 2024</li>
    <li>Description: We need a RaspberryPi 5, a servomotor, a ultrasound sensor and another device compatible with ROS2 Humble. Using the ROS2 Humble framework, a node is created that contains the "action interface" where the type of message that the client must send to the server is defined. For the server part, the RaspberryPi 5 is used with the Raspberry Pi OS operating system, based on Debian, so it does not allow ROS2 to be installed. To solve this, a Docker container with an Ubuntu image with ROS2 Humble installed among other dependencies is used. In order to control the sensors and actuators, the Libgpiod library is used. Once this package is launched, it waits to receive a client request. The client is located on another device with ROS2 Humble installed and networked with the Raspberry Pi 5, from a "launch file" that runs this node and with which it is possible to modify its parameters. Once the message has been sent, the client will begin to receive constant feedback with the current positions of the servo and the distances measured by the ultrasonic sensor. With this data, it generates a window where the different signals that have been received during the entire time of operation are displayed. scanning, for the latter OpenCV is used. </li>
</ul>

### Español: 
<ul>
    <li>Fechas: Abril 2024</li>
    <li>Descripción: Se parte de una RaspberryPi 5, un servomotor y un sesor de ultrasonidos, ademas se necesita otro dispositivo compatible con ROS2 Humble. Empleando el framework ROS2 Humble se crea un nodo que contiene la "action interface" donde se define el tipo de mensaje que el el cliente debe enviar al servidor. Para la parte del servidor, se emplea la RaspberryPi 5 con el sistema operativo Raspberry Pi OS, basado en Debian por lo que no permite instalar ROS2, para solucionar esto se emplea una contenedor Docker con imagen de Ubuntu con ROS2 Humble instalado entre otras dependencias. Para poder controlar los sensores y actuadores se emplea la libreria Libgpiod. Una vez lanzado este paquete este espera a recibir la peticion de un cliente. En otro dispositivo con ROS2 Humble instalado y en red con la Raspberry Pi 5 se ubica el cliente, desde un "launch file" que ejecuta este nodo y con el que es posible modificar sus parametros. El cliente una vez enviado el mensaje empezara a recibir feedback constante con las posiciones actuales del servo y la distancias medidas por el sensor de ultrasonidos, con estos datos este genera una ventana donde se pintan las distintas señales que se han recibido durante todo el tiempo de escaneo, para esto ultimo se emplea OpenCV.</li>
</ul>
   
![foto](https://github.com/asier-vega-gutierrez/ROS2_RPI5_Radar/blob/main/doc/EsquemaGeneral.png)

</li>
    <li>Resultados:</li>
</ul>

![foto](https://github.com/asier-vega-gutierrez/ROS2_RPI5_Radar/blob/main/doc/Demostracion.png)
