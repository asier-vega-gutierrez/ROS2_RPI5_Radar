#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "rpi5_radar_action_interface/action/rpi5radar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rpi5_radar_visual{

    //Creamos un nodo
    class Rpi5RadarActionClient : public rclcpp::Node{
        public:

            //ATRIBUTOS
            //Indicamos que interfaz de accion vamos a usar
            using Rpi5radar = rpi5_radar_action_interface::action::Rpi5radar;
            using GoalHandle = rclcpp_action::ClientGoalHandle<Rpi5radar>;
            //Constantes para hacer la request
            const int servo_waypoints = 50;
            const int servo_start = 0.0;
            const int servo_end = 180.0;
            const int loop_speed = 10.0;

            //METODOS
            //Constructor que incializa el cliente de accion
            explicit Rpi5RadarActionClient(const rclcpp::NodeOptions & options): Node("rpi5_action_client", options){
                this->client_ptr_ = rclcpp_action::create_client<Rpi5radar>( //la interfaz de .accion
                this, //este nodo
                "rpi5_radar"); //nombre de la accion
                //este timer envia cada 500ms una solo goal
                this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&Rpi5RadarActionClient::send_goal, this));
            }
            //funcion que se ejecuta cada 500ms para enviar el goal
            void send_goal(){
                using namespace std::placeholders;
                //cancelamos el timer solo se envia un solo goal
                this->timer_->cancel(); 
                //Comprobamos que el servidor este levantado
                if (!this->client_ptr_->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();
                }
                //Estabelcemos el mensaje goal para rellenarlo
                auto goal_msg = Rpi5radar::Goal();
                //Se escribe en la parte de request por que se envia un peticion de objetivo
                goal_msg.servo_start = this->servo_start;
                goal_msg.servo_end = this->servo_end;
                goal_msg.servo_waypoints = this->servo_waypoints;
                goal_msg.loop_speed = this->loop_speed;
                //Realizamo el envio
                RCLCPP_INFO(this->get_logger(), "Sending goal");
                //Tenemos que estabelcer las opciones de la request 
                auto send_goal_options = rclcpp_action::Client<Rpi5radar>::SendGoalOptions();
                //Metodo para recepcionar la respuesta de nuestra request
                send_goal_options.goal_response_callback = std::bind(&Rpi5RadarActionClient::goal_response_callback, this, _1);
                //Metodo para recepcionar el feedback que se va a recibir de forma continua
                send_goal_options.feedback_callback = std::bind(&Rpi5RadarActionClient::feedback_callback, this, _1, _2); 
                //Metodo para recepcionar la resultado
                send_goal_options.result_callback = std::bind(&Rpi5RadarActionClient::result_callback, this, _1);
                //Ahora que ya hemos estabelcido la manera de recibir los mensajes de respuesta enviamos la request
                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            }

        private:

            //ATRIBUTOS
            rclcpp_action::Client<Rpi5radar>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;
            //Esta variables son para poder mostrar el scaneo de forma actualizada
            double *acumulated_feedback_ultrasonic_read = new double[this->servo_waypoints*2];
            double *acumulated_feedback_servo_degree = new double[this->servo_waypoints*2];
            int feedback_id{0};

            //METODOS
            //Metodo para recepcionar la respuesta de nuestra request
            void goal_response_callback(const GoalHandle::SharedPtr & goal_handle){
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            }
            //Metodo para recepcionar el feedback que se va a recibir de forma continua
            void feedback_callback(GoalHandle::SharedPtr,const std::shared_ptr<const Rpi5radar::Feedback> feedback){
                //Pintamos el feedbakc por consola
                std::stringstream ss;
                ss << "Feedback received: ";
                ss << feedback->ultrasonic_read << " ";
                ss << feedback->servo_degree << " ";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                //Acumulamos el feedback en esta variabels apre despues pintarlo
                acumulated_feedback_ultrasonic_read[feedback_id] = feedback->ultrasonic_read;
                acumulated_feedback_servo_degree[feedback_id] = feedback->servo_degree;
                //Pintamos el esacaneo hasta ahora
                print_radar(acumulated_feedback_ultrasonic_read,acumulated_feedback_servo_degree, 1);
                //sumamos uno al id del feedback
                feedback_id++;
            }
            //Metodo para recepcionar la resultado
            void result_callback(const GoalHandle::WrappedResult & result){
                //Comprobamos el resultado enviado
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        feedback_id = 0;
                        return;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                        feedback_id = 0;
                        return;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        feedback_id = 0;
                        return;
                }
                //Pintamos el resultado enviado
                std::stringstream ss;
                ss << "Result received: ";
                for (auto number : result.result->ultrasonic_reads) {
                    ss << number << " ";
                }
                for (auto number : result.result->servo_degrees) {
                    ss << number << " ";
                }
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                //Repintamos el escaneo completo
                print_radar(result.result->ultrasonic_reads, result.result->servo_degrees, 0);
                //Aciones de fin de nodo
                delete[] acumulated_feedback_ultrasonic_read;
                delete[] acumulated_feedback_servo_degree;
                rclcpp::shutdown();
            }
            //Metodo para pintar el radar
            void print_radar(auto distances, auto degrees, bool scan){
                //Colores
                cv::Scalar red_color(0, 0, 255);
                cv::Scalar blue_color(255, 0, 0);
                cv::Scalar white_color(255, 255, 255);
                // Create a black image
                cv::Mat image = cv::Mat::zeros(720, 1280, CV_8UC3);
                // Draw a filled circle
                cv::Point radar_point(426, 360);
                cv::circle(image, radar_point, 5, red_color, cv::FILLED);
                for (int i = 0; i < this->servo_waypoints*2; i++){
                    //Obtenemos el angulo en radianes y la distancia
                    double angle_print = (degrees[i]) * (CV_PI / 180.0);
                    double distance_print = distances[i];
                    if (distance_print == -1){
                        distance_print = 300;
                    }
                    //Calculamos el punto con respecto a radar
                    int end_x = radar_point.x + static_cast<int>(distance_print * std::sin(angle_print));
                    int end_y = radar_point.y + static_cast<int>(distance_print * std::cos(angle_print));
                    cv::Point endpoint(end_x, end_y);
                    //Pintamos una linea en funcion de en que direccion valla el servo (ida o vuelta)
                    if (i < this->servo_waypoints){
                        cv::line(image, radar_point, endpoint, white_color, 1);
                    }else{
                        cv::line(image, radar_point, endpoint, blue_color, 1);
                    }
                }
                //Mostramos por pantalla la imagen
                cv::namedWindow("Black Image", cv::WINDOW_AUTOSIZE);
                cv::imshow("Black Image", image);
                //En funcion de si estamo escaneando o no la imagen depues de pintarse se para o no
                if (scan == 1){ 
                    cv::waitKey(1);
                }else{
                    cv::waitKey(0);
                }
            }
    };// class Rpi5RadarActionClient
}// namespace rpi5_radar_visual 


RCLCPP_COMPONENTS_REGISTER_NODE(rpi5_radar_visual::Rpi5RadarActionClient)
