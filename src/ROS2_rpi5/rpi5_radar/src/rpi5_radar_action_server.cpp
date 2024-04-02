#include <functional>
#include <memory>
#include <thread>
#include <gpiod.h>

#include "rpi5_radar_action_interface/action/rpi5radar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"



namespace rpi5_radar{

    //Creamo un nodo
    class Rpi5RadarActionServer : public rclcpp::Node{
        public:

            //Indicamos que interfaz de accion vamos a usar
            using Rpi5radar = rpi5_radar_action_interface::action::Rpi5radar;
            using GoalHandle = rclcpp_action::ServerGoalHandle<Rpi5radar>;

            //Con este constrcutor inicamos el nodo como action server
            explicit Rpi5RadarActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("rpi5_action_server", options){
                using namespace std::placeholders;
                //RPI5
                setup_rpi5();
                //El action server
                this->action_server_ = rclcpp_action::create_server<Rpi5radar>( //la interfaz de .action
                this, //este nodo
                "rpi5_radar", //nombre de la accion
                std::bind(&Rpi5RadarActionServer::handle_goal, this, _1, _2), //Calback para goals
                std::bind(&Rpi5RadarActionServer::handle_cancel, this, _1), //Callback para cancels
                std::bind(&Rpi5RadarActionServer::handle_accepted, this, _1)); //Callback para accepts
            }

        private:

            //Atributos
            //Action server
            rclcpp_action::Server<Rpi5radar>::SharedPtr action_server_;
            //RPI5
            //El tipo de chip en Raspberrypi5
            const char *chipname = "gpiochip4";
            struct gpiod_chip *chip;
            //Lineas
            struct gpiod_line *line_echo; //pin 5
            struct gpiod_line *line_trigger; //pin 6
            struct gpiod_line *line_servo; //pin 14
            //Constantes del servo
            const int PULSE_WIDTH_MAX = 2500;
            const int PULSE_WIDTH_MIN = 300;

            //Funcion para establecer el tipo de linea para cada linea recuperada
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
            double servo_move(gpiod_line *line_servo, double servo_goal){
                RCLCPP_INFO(this->get_logger(),"Moving servo to %f", servo_goal);
                //Calculamos el ancho del pulso para nuestro servo
                int pulse_width_microseconds = (servo_goal * ((PULSE_WIDTH_MAX)/180)) + PULSE_WIDTH_MIN;
                //Si ocurriese que nos pasemos de una posicion minima o maxima se corrije
                if (pulse_width_microseconds > PULSE_WIDTH_MAX){
                    pulse_width_microseconds = PULSE_WIDTH_MAX;
                }
                if (pulse_width_microseconds < PULSE_WIDTH_MIN){
                    pulse_width_microseconds = PULSE_WIDTH_MIN;
                }
                RCLCPP_INFO(this->get_logger(),"Moving servo ms %d", pulse_width_microseconds);
                //Encendemos y pagamos el pulso el tiempo especificado
                gpiod_line_set_value(line_servo, 1);
                usleep(pulse_width_microseconds);
                gpiod_line_set_value(line_servo, 0);
                usleep(20000 - pulse_width_microseconds);
                return servo_goal;
            }
            //Funcion para leer el ultrasonidos
            double ultrasonic_read(gpiod_line *trigger, gpiod_line *echo) {
                //Distancia en mm
                double distance{};
                double temp_distance{};
                //Diferencia de timepo entre senales
                double time {};
                //Velocidad del sonido en mm/s
                double speed_of_sound {343000};
                //Estabelcemos una estrcutura con la que medir el tiempo
                struct timeval start, stop;
                //Estabelcemos un numero de iteraciones para evitar bucles infinitos
                int timeout = 10000;
                //Numero de lecturas para hacer la media 
                int readings[5]{};
                //Sumatorio de las lecturas
                double sum_readings{};
                for (int i{0}; i < std::size(readings); i++){
                    //Lo primero que hacemos es encender y apagar el triger para producir la se�al
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
                    //La distancia total se deve dividir por dos (ida y vuelta de la se�al sonica)
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
            //Metodo para preparar la raspberrypi5
            int setup_rpi5(){	

                //Establecemos el chip
                chip = gpiod_chip_open_by_name(chipname);
                if (!chip) {
                    std::cerr << "Could not open chip." << std::endl;
                    return 1;
                }
                //Recuperamos las lineas
                line_trigger = gpiod_chip_get_line(chip, 6);
                line_servo = gpiod_chip_get_line(chip, 14);
                line_echo = gpiod_chip_get_line(chip, 5);
                if(!(line_echo || line_trigger || line_servo)){
                    std::cerr << "Could not get line." << std::endl;
                }
                //Establecemos las lineas de salida como output
                if (request_line(line_trigger, "example1", GPIOD_LINE_REQUEST_DIRECTION_OUTPUT) == -1) {
                    std::cerr << "Could not set output line." << std::endl;
                    return 1;
                }
                //Establecemos las lineas de salida como output
                if (request_line(line_servo, "example1", GPIOD_LINE_REQUEST_DIRECTION_OUTPUT) == -1) {
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
            //Metodo para liberar la raspberry
            int release_rpi5(){
                return 0;
            }


            //Calback para goals  
            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Rpi5radar::Goal> goal){
                RCLCPP_INFO(this->get_logger(), "Received goal request with order %f and %f", goal->servo_start,goal->servo_end);
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
            //Callback para cancels
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle){
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }
            //Callback para accepts
            void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
                using namespace std::placeholders;
                //this needs to return quickly to avoid blocking the executor, so spin up a new thread
                
                std::thread{std::bind(&Rpi5RadarActionServer::execute, this, _1), goal_handle}.detach();
                
                //std::thread execution_thread(&Rpi5RadarActionServer::execute, this, goal_handle);
                //execution_thread.join();

            }
            //Como algunas acciones se deven actualizar de manera casi inmediata se hacen en un hilo aparte
            //Esto devuelve la lectura del ultrasonidos y la posicion del servo cad segundo
            void execute(const std::shared_ptr<GoalHandle> goal_handle){
                RCLCPP_INFO(this->get_logger(), "Executing goal");
                //Establecemos la velocidad de loops
                rclcpp::Rate loop_rate(1);
                const auto goal = goal_handle->get_goal();
                //Definir el feedbak a dar
                auto feedback = std::make_shared<Rpi5radar::Feedback>();
                auto &ultrasonic_feedback = feedback->ultrasonic_read;
                auto &servo_feedback = feedback->servo_degree;
                //Colocamos el servo en la posicion incial
                servo_move(line_servo, goal->servo_start);
                //Hacemos ya una lectura incial
                ultrasonic_feedback = ultrasonic_read(line_trigger, line_echo);
                servo_feedback = goal->servo_start;
                //Definir el resultado
                auto result = std::make_shared<Rpi5radar::Result>();
                auto &ultrasonic_result = result->ultrasonic_reads;
                auto &servo_result = result->servo_degrees;
                //Iteramos hasta conseguir el objetivo del request
                for (int i = 0; (i < goal->servo_waypoints) && rclcpp::ok(); ++i) {
                    //Si se produce una calcelacion del cleinte se deve enviar el resultado igualmente
                    if (goal_handle->is_canceling()) {
                        //Cojemos el ultimo feedback
                        ultrasonic_result.push_back(ultrasonic_feedback);
                        servo_result.push_back(servo_feedback);
                        //Enviamos el resutlado con la cancelacion
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal canceled");
                        return;
                    }
                    //El servo se mueve a la posicion actual + el cacho hasta el siguiente waypoint
                    //Actualizamo el feedback
                    servo_feedback = servo_move(line_servo, servo_feedback + (goal->servo_end/goal->servo_waypoints));
                    ultrasonic_feedback = ultrasonic_read(line_trigger, line_echo);
                    //Publicamos el feedback
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Publish feedback");
                    //Añadimos los valores de este feedback a la secuencia de resultado
                    ultrasonic_result.push_back(ultrasonic_feedback);
                    servo_result.push_back(servo_feedback);
                    //Dormimos el tiempo indicado en rclcpp::Rate
                    loop_rate.sleep();
                }
                //Comprobamos si se ha complido la meta
                if (rclcpp::ok()) {
                    //Enviamos el resultado
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                }
        }
    };// class Rpi5RadarActionServer

}// namespace rpi5_radar

RCLCPP_COMPONENTS_REGISTER_NODE(rpi5_radar::Rpi5RadarActionServer)