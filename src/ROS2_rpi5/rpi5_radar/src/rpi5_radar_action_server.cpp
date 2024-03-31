#include <functional>
#include <memory>
#include <thread>

#include "rpi5_radar_action_interface/action/rpi5radar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rpi5_radar{
    //Creamo un nodo como siempre
    class Rpi5RadarActionServer : public rclcpp::Node{
        public:
        //Indicamos que interfaz de accion vamos a usar
        using Rpi5radar = rpi5_radar_action_interface::action::Rpi5radar;
        using GoalHandle = rclcpp_action::ServerGoalHandle<Rpi5radar>;

        //Con este constrcutor inicamos el nodo como action server
        explicit Rpi5RadarActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("rpi5_action_server", options){
            using namespace std::placeholders;
            //El action server
            this->action_server_ = rclcpp_action::create_server<Rpi5radar>( //la interfaz de .action
            this, //este nodo
            "fibonacci", //nombre de la accion
            std::bind(&Rpi5RadarActionServer::handle_goal, this, _1, _2), //Calback para goals
            std::bind(&Rpi5RadarActionServer::handle_cancel, this, _1), //Callback para cancels
            std::bind(&Rpi5RadarActionServer::handle_accepted, this, _1)); //Callback para accepts
        }

        private:
        rclcpp_action::Server<Rpi5radar>::SharedPtr action_server_;

        //Calback para goals  
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Rpi5radar::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
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
            //Hacemos ya una lectura incial
            ultrasonic_feedback = 0.0;
            servo_feedback = 0.0;
            //Definir el resultado
            auto result = std::make_shared<Rpi5radar::Result>();
            auto &ultrasonic_result = result->ultrasonic_reads;
            auto &servo_result = result->servo_deegres;
            //Iteramos hasta conseguir el objetivo del request
            for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
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
                //Actualizamo el feedback
                ultrasonic_feedback = ultrasonic_feedback + 1.0;
                servo_feedback = servo_feedback + 1.0;
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