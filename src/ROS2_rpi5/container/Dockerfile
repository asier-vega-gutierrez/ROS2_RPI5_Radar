# Partimos de la imagen base
FROM ros:humble-ros-base
# Actualizamos
RUN apt update 
RUN apt install curl -y
RUN apt install apt-utils -y 
RUN apt install net-tools -y 
RUN apt install nmap -y 
RUN apt install xauth -y
RUN apt install git -y
RUN apt-get install libgpiod2 libgpiod-dev -y
RUN apt install gpiod -y

#Establecemos el source
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "sleep", "infinity"]
