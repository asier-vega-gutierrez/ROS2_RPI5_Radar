#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <csignal>


volatile sig_atomic_t signal_received = 0;
//Iterrupcion para poder para el programa con crtl + c
void sigint_handler(int signal) {
  signal_received = signal;
}

//Funcion apra estabcer el tipo de linea para cada linea recuperada
int request_line(gpiod_line *line, const std::string &consumer, int direction) {
  //Establecemos la configuracion para hacer la peticion
  gpiod_line_request_config config = {.consumer = consumer.c_str(),.request_type = direction};
  //Realizamo la peticion para establecer la linea
  int ret = gpiod_line_request(line, &config, direction);
  if (ret == -1) {
    std::cerr << "Could not set line." << std::endl;
  }
  return ret;
}

//Funcion para leer el ultrasonidos
float ultrasonic_read(gpiod_line *trigger, gpiod_line *echo) {

  //Distancia en mm
  float distance{};
  float temp_distance{};
  //Diferencia de timepo entre señales
  double time {};
  //Velocidad del sonido en mm/s
  float speed_of_sound {343000};
  //Estabelcemos una estrcutura con la que medir el tiempo
  struct timeval start, stop;
  //Estabelcemos un numero de iteraciones para evitar bucles infinitos
  int timeout = 10000;
  //Numero de lecturas para hacer la media 
  int readings[5]{};
  //Sumatorio de las lecturas
  float sum_readings{};

  for (int i{0}; i < std::size(readings); i++){
    //Lo primero que hacemos es encender y apagar el triger para producir la señal
    gpiod_line_set_value(trigger, 1);
    usleep(10);
    gpiod_line_set_value(trigger, 0);

    //Esperamos a que echo de una señal de que detecta el triger, e iniciamos el contador
    while (gpiod_line_get_value(echo) == 0 && timeout > 0) {
      gettimeofday(&start, NULL);
      timeout--;
    }

    //Esperamos a que la señal se acabe para para el contador
    while (gpiod_line_get_value(echo) == 1 && timeout > 0) {
      gettimeofday(&stop, NULL);
      timeout--;
    }

    //claculamos la diferencia en segundo y microsegundos
    time = (stop.tv_sec - start.tv_sec) + (stop.tv_usec - start.tv_usec) / 1000000.0;

    //La distancia total se deve dividir por dos (ida y vuelta de la señal sonica)
    temp_distance = (speed_of_sound * time) / 2.0;

    //Almacenamos las lecturas
    readings[i] = temp_distance;

    //Añadimos un pequeño delay entre lecturas
    usleep(1000);
  }

  //Sumamos todas las distancias y hacemos la media de todas ellas
  for (int value : readings){
    sum_readings = sum_readings + value;
  }
  distance = sum_readings/std::size(readings);
  
  //Si la distancia pasa los limites del sonser devuvle un -1
  if (distance > 300.0){
    distance = -1;
  }
  if (distance < 5.0){
    distance = -1;
  }

  //Devolvemos los mm medidos
  return distance;
}


int main(int argc, char **argv) {

  //El tipo de chip en Raspberrypi5 es el chip4
  const char *chipname = "gpiochip4";
  //Lineas
  struct gpiod_chip *chip;
  struct gpiod_line *line_echo;
  struct gpiod_line *line_trigger;
  struct gpiod_line_event *event;


  //Establecemos el chip
  chip = gpiod_chip_open_by_name(chipname);
  if (!chip) {
    std::cerr << "Could not open chip." << std::endl;
    return 1;
  }

  //Recuperamos las lineas
  line_echo = gpiod_chip_get_line(chip, 5);
  line_trigger = gpiod_chip_get_line(chip, 6);
  if(!(line_echo || line_trigger)){
    std::cerr << "Could not get line." << std::endl;
  }

  //Establecemos las lineas de salida como output
  if (request_line(line_trigger, "example1", GPIOD_LINE_REQUEST_DIRECTION_OUTPUT) == -1) {
    std::cerr << "Could not set output line." << std::endl;
    return 1;
  }
  //Establecemos las lineas de entrada como input
  if (request_line(line_echo, "example1", GPIOD_LINE_REQUEST_DIRECTION_INPUT) == -1) {
    std::cerr << "Could not set output line." << std::endl;
    return 1;
  }

  //Habilitamos la interrupcion
  signal(SIGINT, sigint_handler);
  std::cout << "Press CTRL-C to exit." << std::endl;

  //Bucle principal
  while (!signal_received) {

    float distance = ultrasonic_read(line_trigger, line_echo);
    std::cout << distance << std::endl;

  }

  //Ponemos las salidas a 0
  gpiod_line_set_value(line_trigger, 0);

  //Liberamos la lineas
  gpiod_line_release(line_trigger);
  gpiod_line_release(line_echo);
  gpiod_chip_close(chip);
  return 0;
}