#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <csignal> //para habilitar la señal de interrupcion


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

const int debounceTime = 50000;
//Metodo para eliminarl el rebote de botones
bool debounceButton(gpiod_line *line) {
  //Estructura para especificar el tiempo de espera
  struct timespec ts = {.tv_sec = 0, .tv_nsec = debounceTime};
  //Obtenemos el estado actual del boton
  bool lastState = gpiod_line_get_value(line);
  //Esperamos al tiempo de rebote establecido
  nanosleep(&ts, NULL);
  //Obtenemos el estado del botn despues de espera el tiempo de rebote
  bool currentState = gpiod_line_get_value(line);
  //Devolvemos un valor u otro en funcion de su estado
  return currentState && !lastState;
}

int main(int argc, char **argv) {

  //El tipo de chip en Raspberrypi5 es el chip4
  const char *chipname = "gpiochip4";
  //Lineas
  struct gpiod_chip *chip;
  struct gpiod_line *line_red_led;    
  struct gpiod_line *line_yellow_led; 
  struct gpiod_line *line_button;
  //Variables
  int state_button {};
  int state_color {true};

  //Establecemos el chip
  chip = gpiod_chip_open_by_name(chipname);
  if (!chip) {
    std::cerr << "Could not open chip." << std::endl;
    return 1;
  }

  //Recuperamos las lineas
  line_red_led = gpiod_chip_get_line(chip, 21);
  line_yellow_led = gpiod_chip_get_line(chip, 20);
  line_button = gpiod_chip_get_line(chip, 16);
  if(!(line_red_led or line_yellow_led or line_button)){
    std::cerr << "Could not get line." << std::endl;
  }

  //Establecemos las lineas de salida como output
  if (request_line(line_red_led, "example1", GPIOD_LINE_REQUEST_DIRECTION_OUTPUT) == -1 || 
    request_line(line_yellow_led, "example1", GPIOD_LINE_REQUEST_DIRECTION_OUTPUT) == -1) {
    std::cerr << "Could not set output line." << std::endl;
    return 1;
  }
  //Establecemos las lineas de entrada como input
  if (request_line(line_button, "example1", GPIOD_LINE_REQUEST_EVENT_RISING_EDGE) == -1) {
    std::cerr << "Could not set input line." << std::endl;
    return 1;
  }

  //Habilitamos la interrupcion
  signal(SIGINT, sigint_handler);
  std::cout << "Press CTRL-C to exit." << std::endl;

  //Bucle principal
  while (!signal_received) {

    state_button = debounceButton(line_button);

    //Este programa enciende una led u otro si pulsamo el boton
    if (state_button == 1) {
      if (state_color == true){
        gpiod_line_set_value(line_red_led, 1);
        gpiod_line_set_value(line_yellow_led, 0);
        state_color = false;
      }else if (state_color == false) {
        gpiod_line_set_value(line_red_led, 0);
        gpiod_line_set_value(line_yellow_led, 1);
        state_color = true;
      }
    }
  }

  //Ponemos las salidas a 0
  gpiod_line_set_value(line_red_led, 0);
  gpiod_line_set_value(line_yellow_led, 0);

  //Liberamos la lineas
  gpiod_line_release(line_red_led);
  gpiod_line_release(line_yellow_led);
  gpiod_line_release(line_button);
  gpiod_chip_close(chip);
  return 0;
}