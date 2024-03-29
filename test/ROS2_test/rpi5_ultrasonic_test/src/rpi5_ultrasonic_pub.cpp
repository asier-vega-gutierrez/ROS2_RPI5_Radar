#include <chrono>
#include <memory>

#include <gpiod.h>
#include "rclcpp/rclcpp.hpp"
#include "rpi5_radar/msg/ultrasonic.hpp" //incluimos el mensaje custom 

using namespace std::chrono_literals;


//El tipo de chip en Raspberrypi5
const char *chipname = "gpiochip4";
struct gpiod_chip *chip;
//Lineas
struct gpiod_line *line_echo; //pin 5
struct gpiod_line *line_trigger; //pin 6

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

int setup_rpi5(){	

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
	return 0;
}

//Funcion para leer el ultrasonidos
float ultrasonic_read(gpiod_line *trigger, gpiod_line *echo) {

	//Distancia en mm
	float distance{};
	float temp_distance{};
	//Diferencia de timepo entre senales
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
		//Lo primero que hacemos es encender y apagar el triger para producir la senal
		gpiod_line_set_value(trigger, 1);
		usleep(10);
		gpiod_line_set_value(trigger, 0);

		//Esperamos a que echo de una senal de que detecta el triger, e iniciamos el contador
		while (gpiod_line_get_value(echo) == 0 && timeout > 0) {
			gettimeofday(&start, NULL);
			timeout--;
		}

		//Esperamos a que la senal se acabe para para el contador
		while (gpiod_line_get_value(echo) == 1 && timeout > 0) {
			gettimeofday(&stop, NULL);
			timeout--;
		}

		//claculamos la diferencia en segundo y microsegundos
		time = (stop.tv_sec - start.tv_sec) + (stop.tv_usec - start.tv_usec) / 1000000.0;

		//La distancia total se deve dividir por dos (ida y vuelta de la senal sonica)
		temp_distance = (speed_of_sound * time) / 2.0;

		//Almacenamos las lecturas
		readings[i] = temp_distance;

		//Anadimos un pequeno delay entre lecturas
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

//Creamon un nodo
class UltrasoniReadPublisher : public rclcpp::Node{

  	public:
		UltrasoniReadPublisher(): Node("ultrasonic_dist_publisher"){
			//Creamos un publicador
			publisher_ =this->create_publisher<rpi5_radar::msg::Ultrasonic>("ultrasonic_read", 10);
			//Funcion de callbakc para enviar el mensaje
			auto publish_msg = [this]() -> void {
				//Creamos un mensaje del tipo de nuestra interfaz
				auto message = rpi5_radar::msg::Ultrasonic();

				message.dist = ultrasonic_read(line_trigger, line_echo);

				std::cout << "Publishing ultrasonic dist:" << message.dist << std::endl;

				//Publicamos el mensaje
				this->publisher_->publish(message);
			};
			//Timer para llmar a la funcion
			timer_ = this->create_wall_timer(1s, publish_msg);
    	}

	private:
		//pulbicador con nuetra interfaz de mensaje
		rclcpp::Publisher<rpi5_radar::msg::Ultrasonic>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char *argv[]){
	
	//RPI5
	//Llammos a los metodos pertinentes
	setup_rpi5();

	//ROS2
	//Iniciamos la libreria
	rclcpp::init(argc, argv);
	//Inicimos el nodo y lo mantenmos constantemente
	rclcpp::spin(std::make_shared<UltrasoniReadPublisher>());
	//Al hacer crtl+c paramos la libreria
	rclcpp::shutdown();

	//RPI5
	//Ponemos las salidas a 0
	gpiod_line_set_value(line_trigger, 0);
	//Liberamos la lineas
	gpiod_line_release(line_trigger);
	gpiod_line_release(line_echo);
	gpiod_chip_close(chip);

	return 0;
}