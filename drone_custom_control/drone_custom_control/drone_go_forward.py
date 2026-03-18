#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget, State

class DroneUpAndForward(Node):
    def __init__(self):
        super().__init__('drone_up_and_forward')
        
        self.pub = self.create_publisher(
            PositionTarget, 
            '/uav1/mavros/setpoint_raw/local', 
            10
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/uav1/mavros/state',
            self.state_callback,
            10
        )
        
        self.current_state = State()
        self.ready = False
        
        # Timer principal (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Config do setpoint (igual à versão anterior, mas só publica quando ready)
        self.target = PositionTarget()
        self.target.header.frame_id = 'local_origin'
        self.target.coordinate_frame = 1  # FRAME_LOCAL_NED
        
        # Type mask: posição z + velocidades xy (recomendado com MRS/MPC)
        self.target.type_mask = (
            PositionTarget.IGNORE_PX +
            PositionTarget.IGNORE_PY +
            PositionTarget.IGNORE_AFX +
            PositionTarget.IGNORE_AFY +
            PositionTarget.IGNORE_AFZ +
            PositionTarget.IGNORE_YAW
        )  # ≈ 1475
        
        self.target.position.z = 2.0 
        self.target.position.x = 2.0 # altura desejada
        self.target.velocity.x = 0.0       # velocidade pra frente
        self.target.velocity.y = 0.0
        self.target.velocity.z = 0.0
        self.target.yaw = 0.0
        self.target.yaw_rate = 0.0
        
        self.get_logger().info('Node iniciado. Aguardando drone pronto (connected + armed + OFFBOARD)...')

    def state_callback(self, msg: State):
        self.current_state = msg
        
        # Condição de "pronto" – ajusta se quiser mais rigoroso
        if (msg.connected and 
            msg.armed and 
            msg.mode == "OFFBOARD"):
            if not self.ready:
                self.get_logger().info('Drone PRONTO! connected + armed + OFFBOARD → começando setpoints')
                self.ready = True

    def timer_callback(self):
        if not self.ready:
            # Ainda não pronto → não publica nada (ou publica setpoint neutro se quiser)
            return
        
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.target)
        # self.get_logger().debug('Setpoint publicado')  # descomenta pra debug

def main(args=None):
    rclpy.init(args=args)
    node = DroneUpAndForward()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parando node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()