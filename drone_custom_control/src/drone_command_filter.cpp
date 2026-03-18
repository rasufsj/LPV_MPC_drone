#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DroneCommandFilter : public rclcpp::Node {
public:
  DroneCommandFilter() : Node("drone_command_filter") {
    pub_setpoint_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/uav1/mavros/setpoint_raw/local", 10);

    sub_command_ = this->create_subscription<std_msgs::msg::Float64>(
        "/drone_command", 10,
        std::bind(&DroneCommandFilter::commandCallback, this, std::placeholders::_1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/uav1/mavros/local_position/pose", 10,
        std::bind(&DroneCommandFilter::poseCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&DroneCommandFilter::publishSetpoint, this));

    RCLCPP_INFO(this->get_logger(), "DroneCommandFilter iniciado");
    RCLCPP_INFO(this->get_logger(), "Escutando /drone_command (Float64: velocidade m/s)");
  }

private:
  void commandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    vx_desired_ = msg->data;
    command_duration_sec_ = 5.0;  // tempo em segundos – ajuste aqui
    command_start_time_ = this->now();
    is_in_command_phase_ = true;

    RCLCPP_INFO(this->get_logger(), "Comando recebido: vx = %.2f m/s por %.1f s",
                vx_desired_, command_duration_sec_);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    // Atualiza hold apenas quando está em hover puro (nenhuma fase ativa)
    if (!is_in_command_phase_ && !is_decelerating_) {
      hold_position_ = current_pose_.pose.position;
    }
  }

  void publishSetpoint() {
    auto now = this->now();
    mavros_msgs::msg::PositionTarget target;
    target.header.stamp = now;
    target.header.frame_id = "local_origin";
    target.coordinate_frame = 1;  // LOCAL_NED – teste 3 se rejeitar

    // FORÇA vz = 0 SEMPRE (anti-subida infinita)
    target.velocity.z = 0.0;

    if (is_in_command_phase_) {
      double elapsed = (now - command_start_time_).seconds();

      if (elapsed >= command_duration_sec_) {
        is_in_command_phase_ = false;
        is_decelerating_ = true;
        deceleration_start_time_ = now;
        RCLCPP_INFO(this->get_logger(), "Comando finalizado → desacelerando (vel zero por 2s)");
      } else {
        // Modo velocidade normal
        target.type_mask = 8160;
        target.velocity.x = vx_desired_;
        target.velocity.y = 0.0;
        target.position.z = current_pose_.pose.position.z;  // segura altura atual
        target.yaw = 0.0;
        target.yaw_rate = 0.0;
      }
    } else if (is_decelerating_) {
      double dec_elapsed = (now - deceleration_start_time_).seconds();

      target.type_mask = 1991;
      target.velocity.x = 0.0;
      target.velocity.y = 0.0;
      target.position.z = current_pose_.pose.position.z;
      target.yaw = 0.0;
      target.yaw_rate = 0.0;

      if (dec_elapsed >= 2.0) {
        is_decelerating_ = false;
        hold_position_ = current_pose_.pose.position;
        RCLCPP_INFO(this->get_logger(), "Desaceleração concluída → HOVER em (%.2f, %.2f, %.2f)",
                    hold_position_.x, hold_position_.y, hold_position_.z);
      }
    } else {
      // Modo HOVER final
      target.type_mask = 0;
      target.position.x = hold_position_.x;
      target.position.y = hold_position_.y;
      target.position.z = hold_position_.z;  // SEM offset fixo – usa exatamente onde parou
      target.yaw = 0.0;
      target.yaw_rate = 0.0;

      static bool logged = false;
      if (!logged) {
        RCLCPP_INFO(this->get_logger(), "Hover estável (type_mask=0) em (%.2f, %.2f, %.2f)",
                    hold_position_.x, hold_position_.y, hold_position_.z);
        logged = true;
      }
    }

    pub_setpoint_->publish(target);
  }

  // Variáveis
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_command_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_in_command_phase_ = false;
  bool is_decelerating_ = false;
  double vx_desired_ = 0.0;
  double command_duration_sec_ = 5.0;
  rclcpp::Time command_start_time_;
  rclcpp::Time deceleration_start_time_;
  geometry_msgs::msg::Point hold_position_;
  geometry_msgs::msg::PoseStamped current_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneCommandFilter>());
  rclcpp::shutdown();
  return 0;
}