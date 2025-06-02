#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.msg import Lifter
from std_msgs.msg import Int8
import gpiod

class LifterNode(Node):
    def __init__(self):
        super().__init__('lifter_control_node')
        
        # Configuración de GPIO (ajusta los números según tu hardware)
        self.GPIO_DIR = 149  # GPIO29 (Dirección: 0=arriba, 1=abajo)
        self.GPIO_EN = 12    # GPIO37 (Enable: 1=activado, 0=inactivo)
        self.GPIO_BUTTON_UP = 216   # GPIO7 (Sensor arriba)
        self.GPIO_BUTTON_DOWN = 77  # GPIO38 (Sensor abajo)

        # Inicializar GPIO
        self.chip = gpiod.Chip('gpiochip0')
        self.dir_line = self.chip.get_line(self.GPIO_DIR)
        self.en_line = self.chip.get_line(self.GPIO_EN)
        self.up_sensor = self.chip.get_line(self.GPIO_BUTTON_UP)
        self.down_sensor = self.chip.get_line(self.GPIO_BUTTON_DOWN)

        # Configurar pines
        self.dir_line.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self.en_line.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        self.up_sensor.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_IN)
        self.down_sensor.request(consumer='lifter', type=gpiod.LINE_REQ_DIR_IN)

        # Estados
        self.ARRIBA = 0
        self.ABAJO = 1
        self.DETENIDO = 2
        self.current_status = self.DETENIDO  # Estado inicial

        # Suscriptor y temporizador
        self.subscription = self.create_subscription(
            Int8,
            '/lifter_status',
            self.lifter_callback,
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)  # 20 Hz

    def lifter_callback(self, msg):
        """Callback para mensajes ROS que indican el estado deseado."""
        self.current_status = msg.status
        self.get_logger().info(f"Estado recibido: {self.current_status}")

    def timer_callback(self):
        """Verifica sensores y controla el motor con tiempo de sobreimpulso."""
        # Leer sensores (True = activado, False = inactivo)
        up_activated = not self.up_sensor.get_value()
        down_activated = not self.down_sensor.get_value()

        # Lógica de sobreimpulso: Mover por 1 segundo adicional al llegar al límite
        if self.current_status == self.ARRIBA and up_activated:
            if not hasattr(self, 'overrun_start'):  # Primer frame detectado
                self.overrun_start = self.get_clock().now()
                self.get_logger().info("Iniciando sobreimpulso de 1 segundo hacia arriba...")
            
            elapsed = (self.get_clock().now() - self.overrun_start).nanoseconds / 1e9  # Segundos
            if elapsed <= 1.0:  # Mover por 1 segundo
                self.dir_line.set_value(0)  # Dirección: arriba
                self.en_line.set_value(1)   # Motor ON
            else:
                self.en_line.set_value(0)    # Desactivar después de 1 segundo
                del self.overrun_start       # Limpiar variable
                self.get_logger().warn("Sobreimpulso completado. Motor detenido.")
            return

        elif self.current_status == self.ABAJO and down_activated:
            if not hasattr(self, 'overrun_start'):
                self.overrun_start = self.get_clock().now()
                self.get_logger().info("Iniciando sobreimpulso de 1 segundo hacia abajo...")
            
            elapsed = (self.get_clock().now() - self.overrun_start).nanoseconds / 1e9
            if elapsed <= 1.0:
                self.dir_line.set_value(1)  # Dirección: abajo
                self.en_line.set_value(1)   # Motor ON
            else:
                self.en_line.set_value(0)
                del self.overrun_start
                self.get_logger().warn("Sobreimpulso completado. Motor detenido.")
            return

    # Control normal (sin sobreimpulso)
    if self.current_status == self.ARRIBA:
        self.dir_line.set_value(0)  # Dirección: arriba
        self.en_line.set_value(1)   # Motor ON
        self.get_logger().info("Subiendo...")
    elif self.current_status == self.ABAJO:
        self.dir_line.set_value(1)  # Dirección: abajo
        self.en_line.set_value(1)   # Motor ON
        self.get_logger().info("Bajando...")
    else:
        self.en_line.set_value(0)   # Motor OFF
        self.get_logger().info("Detenido.")

    def __del__(self):
        """Liberar GPIO al destruir el nodo."""
        self.dir_line.set_value(0)
        self.en_line.set_value(0)
        self.dir_line.release()
        self.en_line.release()
        self.up_sensor.release()
        self.down_sensor.release()
        self.chip.close()
        self.get_logger().info("GPIO liberado.")

def main(args=None):
    rclpy.init(args=args)
    node = LifterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()