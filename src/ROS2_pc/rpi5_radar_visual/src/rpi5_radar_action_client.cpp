#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rpi5_radar_action_interface/action/rpi5radar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rpi5_radar_visual{
    //Creamos un nodo
    class Rpi5RadarActionClient : public rclcpp::Node{
        public:
            //Indicamos que interfaz de accion vamos a usar
            using Rpi5radar = rpi5_radar_action_interface::action::Rpi5radar;
            using GoalHandle = rclcpp_action::ClientGoalHandle<Rpi5radar>;
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
                goal_msg.servo_start = 0.0;
                goal_msg.servo_end = 100.0;
                goal_msg.servo_waypoints = 15;

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
            rclcpp_action::Client<Rpi5radar>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;

            //Metodo para recepcionar la respuesta de nuestra request
            void goal_response_callback(const GoalHandle::SharedPtr & goal_handle){
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            }
            //Metodo para recepcionar el feedback que se va a recibir de forma continua
            void feedback_callback(
                GoalHandle::SharedPtr,const std::shared_ptr<const Rpi5radar::Feedback> feedback){
                std::stringstream ss;
                ss << "Feedcback received: ";
                ss << feedback->ultrasonic_read << " ";
                ss << feedback->servo_degree << " ";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            }
            //Metodo para recepcionar la resultado
            void result_callback(const GoalHandle::WrappedResult & result){
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        return;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                        return;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        return;
                }
                std::stringstream ss;
                ss << "Result received: ";
                for (auto number : result.result->ultrasonic_reads) {
                    ss << number << " ";
                }
                for (auto number : result.result->servo_degrees) {
                    ss << number << " ";
                }
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                rclcpp::shutdown();
        }
    };  // class Rpi5RadarActionClient

}  // namespace rpi5_radar_visual

RCLCPP_COMPONENTS_REGISTER_NODE(rpi5_radar_visual::Rpi5RadarActionClient)
