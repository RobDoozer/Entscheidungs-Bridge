import rclpy
from rclpy.node import Node
import can
from datetime import datetime

from can_interface import can_definitions as defs

class CANBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')

        self.bus_rx = can.interface.Bus(channel='can0', bustype='socketcan')
        self.bus_tx = can.interface.Bus(channel='can1', bustype='socketcan')

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logfile = open(f'/tmp/can_bridge_log_{timestamp}.csv', 'w')
        self.logfile.write('timestamp,id,data,action\n')

        self.hindernis_status = defs.HINDERNIS_NONE
        self.timer = self.create_timer(0.01, self.bridge_loop)

    def is_bit_set(self, value, bit):
        return (value >> bit) & 1

    def bridge_loop(self):
        msg = self.bus_rx.recv(timeout=0.01)
        if msg:
            data_list = list(msg.data)
            hex_data = ' '.join(f'{b:02X}' for b in data_list)
            self.get_logger().info(f'Empfangen: ID=0x{msg.arbitration_id:X}, Daten={hex_data}')
            self.logfile.write(f'{datetime.now()},{msg.arbitration_id},{data_list},RECEIVED\n')
            self.logfile.flush()

            # Sensorstatus aktualisieren
            if msg.arbitration_id == defs.ID_HINDERNIS:
                self.hindernis_status = msg.data[0]
                self.get_logger().info(f'Hindernisstatus aktualisiert: 0b{self.hindernis_status:08b}')
                return

            # Blockierlogik für Fahrbefehl
            if msg.arbitration_id == defs.ID_FAHRBEFEHL:
                if data_list == defs.FAHRBEFEHL_VOR and self.is_bit_set(self.hindernis_status, defs.HINDERNIS_VORNE_BIT):
                    self.get_logger().warn('Fahrbefehl VOR blockiert wegen Hindernis VORNE')
                    self.logfile.write(f'{datetime.now()},{msg.arbitration_id},{data_list},BLOCKED\n')
                    return
                if data_list == defs.FAHRBEFEHL_RUECK and self.is_bit_set(self.hindernis_status, defs.HINDERNIS_HINTEN_BIT):
                    self.get_logger().warn('Fahrbefehl RÜCK blockiert wegen Hindernis HINTEN')
                    self.logfile.write(f'{datetime.now()},{msg.arbitration_id},{data_list},BLOCKED\n')
                    return

            # Weiterleitung, wenn nicht blockiert
            try:
                new_msg = can.Message(arbitration_id=msg.arbitration_id,
                                      data=msg.data,
                                      is_extended_id=msg.is_extended_id)
                self.bus_tx.send(new_msg)
                self.get_logger().info(f'Gesendet: ID=0x{msg.arbitration_id:X}, Daten={hex_data}')
                self.logfile.write(f'{datetime.now()},{msg.arbitration_id},{data_list},SENT\n')
            except can.CanError as e:
                self.get_logger().error(f'Sendefehler: {e}')
                self.logfile.write(f'{datetime.now()},{msg.arbitration_id},{data_list},ERROR\n')

    def destroy_node(self):
        self.logfile.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
