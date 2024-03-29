#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <csignal> //para habilitar la señal de interrupcion


const int PULSE_WIDTH_MAX = 2500;
const int PULSE_WIDTH_MIN = 300;

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

//Funcion para mover el servo
void servo_move(gpiod_line *line_servo, int servo_goal){
  //Calculamos el ancho del pulso para nuestro servo
  int pulse_width_microseconds = (servo_goal * ((PULSE_WIDTH_MAX)/180)) + PULSE_WIDTH_MIN;
  //Si ocurriese que nos pasemos de una posicion minima o maxima se corrije
  if (pulse_width_microseconds > PULSE_WIDTH_MAX){
    pulse_width_microseconds = PULSE_WIDTH_MAX;
  }
  if (pulse_width_microseconds < PULSE_WIDTH_MIN){
    pulse_width_microseconds = PULSE_WIDTH_MIN;
  }
  //Encendemos y pagamos el pulso el tiempo especificado
  gpiod_line_set_value(line_servo, 1);
  usleep(pulse_width_microseconds);
  gpiod_line_set_value(line_servo, 0);
  usleep(20000 - pulse_width_microseconds);
}

int main(int argc, char **argv) {

  //El tipo de chip en Raspberrypi5 es el chip4
  const char *chipname = "gpiochip4";
  //Lineas
  struct gpiod_chip *chip;
  struct gpiod_line *line_servo;    
  //Variables
  int servo_goal = 180;
  int servo_waypoints = 50; //solo para el modo indirecto

  //Establecemos el chip
  chip = gpiod_chip_open_by_name(chipname);
  if (!chip) {
    std::cerr << "Could not open chip." << std::endl;
    return 1;
  }

  //Recuperamos las lineas
  line_servo = gpiod_chip_get_line(chip, 14);
  if(!(line_servo)){
    std::cerr << "Could not get line." << std::endl;
  }

  //Establecemos las lineas de salida como output
  if (request_line(line_servo, "example1", GPIOD_LINE_REQUEST_DIRECTION_OUTPUT) == -1) {
    std::cerr << "Could not set output line." << std::endl;
    return 1;
  }
  //Establecemos las lineas de entrada como input

  //Habilitamos la interrupcion
  signal(SIGINT, sigint_handler);
  std::cout << "Press CTRL-C to exit." << std::endl;

  //Bucle principal
  while (!signal_received) {
    
    //De forma directa
    //servo_move(line_servo, servo_goal);

    //De forma indirecta
    int intermediate_step = servo_goal / (servo_waypoints + 1);
    int current_position = 0;
    for (int i = 1; i <= servo_waypoints; ++i) {
      current_position += intermediate_step;
      servo_move(line_servo, current_position);
      usleep(100000);
    }

  }

  //Ponemos las salidas a 0
  gpiod_line_set_value(line_servo, 0);

  //Liberamos la lineas
  gpiod_line_release(line_servo);
  gpiod_chip_close(chip);
  return 0;
}