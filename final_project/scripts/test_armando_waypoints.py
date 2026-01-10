#!/usr/bin/env python3
"""
Test script per waypoint Armando con gestione multipli bagagli

Usage:
    ros2 run final_project test_armando_waypoints.py --ros-args -p station:=1 -p num_bags:=3
    
Testa la sequenza baggage handling per un singolo Armando.
Ripete il ciclo collect→lift→release per ogni bagaglio.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class ArmandoWaypointTester(Node):
    def __init__(self):
        super().__init__('armando_waypoint_tester')
        
        # Parametri
        self.declare_parameter('station', 1)
        self.declare_parameter('num_bags', 1)  # numero bagagli
        
        self.station_id = self.get_parameter('station').value
        self.num_bags = self.get_parameter('num_bags').value
        
        # Determina colonna (DX o SX)
        self.is_dx_column = (self.station_id % 2 == 1)
        
        # Namespace Armando
        self.namespace = f'armando_{self.station_id}'
        
        # Publishers
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.namespace}/position_controller/commands',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.namespace}/gripper_controller/commands',
            10
        )
        
        # Waypoint:
        # - Rimossi j2 e j3 negativi che causavano collisione terra
        # - Aggiunta posizione intermedia 'lift' per alzare bagaglio
        
        if self.is_dx_column:
            # Colonna DX (station 1, 3, 5) - yaw=0, guarda +X
            self.waypoints = {
                'home': [0.0, 0.0, 0.0, 0.0],
                'fra2mo': [0.6, -1.6, -0.0, -0.0],       # Posizione bassa verso Fra2mo
                'lift': [0.6, -1.0, 0.0, 0.0],           # Alza bagaglio (posizione intermedia sicura)
                'aruco': [-0.6, -1.6, -0.0, -0.0],       # Posizione bassa verso ArUco
            }
        else:
            # Colonna SX (station 2, 4, 6) - yaw=π, guarda -X
            # Specchio: j0 invertito
            self.waypoints = {
                'home': [0.0, 0.0, 0.0, 0.0],
                'fra2mo': [-0.6, -1.6, -0.0, -0.0],      # j0 invertito
                'lift': [-0.6, -1.0, 0.0, 0.0],          # j0 invertito
                'aruco': [0.6, -1.6, -0.0, -0.0],        # j0 invertito
            }
        
        # Gripper states
        self.gripper_open = [0.02, 0.02]
        self.gripper_closed = [0.003, 0.003]
        
        self.get_logger().info(f'Armando {self.station_id} waypoint tester ready')
        self.get_logger().info(f'Column: {"DX" if self.is_dx_column else "SX"}')
        self.get_logger().info(f'Number of bags to handle: {self.num_bags}')
        
        # Aspetta che publisher siano pronti
        time.sleep(1.0)
    
    def move_to_waypoint(self, waypoint_name, duration=3.0):
        """Muove braccio a waypoint specificato"""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'Unknown waypoint: {waypoint_name}')
            return
        
        wp = self.waypoints[waypoint_name]
        
        msg = Float64MultiArray()
        msg.data = wp
        
        self.get_logger().info(f'Moving to {waypoint_name}: {wp}')
        self.position_pub.publish(msg)
        
        # Aspetta completamento (approssimativo)
        time.sleep(duration)
    
    def control_gripper(self, state, duration=1.5):
        """Controlla gripper (open/closed)"""
        msg = Float64MultiArray()
        
        if state == 'open':
            msg.data = self.gripper_open
            self.get_logger().info('Opening gripper')
        elif state == 'closed':
            msg.data = self.gripper_closed
            self.get_logger().info('Closing gripper')
        else:
            self.get_logger().error(f'Unknown gripper state: {state}')
            return
        
        self.gripper_pub.publish(msg)
        time.sleep(duration)
    
    def handle_single_baggage(self, bag_number):
        """
        Gestisce un singolo bagaglio (ciclo completo)
        
        SEQUENZA:
        1. Vai a Fra2mo
        2. Chiudi pinza (prelievo)
        3. Alza a posizione intermedia
        4. Vai ad ArUco
        5. Apri pinza (rilascio)
        """
        self.get_logger().info(f'--- HANDLING BAGGAGE {bag_number}/{self.num_bags} ---')
        
        # 1. Vai a Fra2mo
        self.get_logger().info(f'[Bag {bag_number}] Moving to Fra2mo (collect position)')
        self.move_to_waypoint('fra2mo', duration=8.0)
        
        # 2. Chiudi pinza (prelievo bagaglio)
        self.get_logger().info(f'[Bag {bag_number}] Closing gripper (pickup)')
        self.control_gripper('closed', duration=3.0)
        
        # 3. Alza bagaglio a posizione intermedia sicura
        self.get_logger().info(f'[Bag {bag_number}] Lifting baggage to safe position')
        self.move_to_waypoint('lift', duration=6.0)
        
        # 4. Vai ad ArUco
        self.get_logger().info(f'[Bag {bag_number}] Moving to ArUco (release position)')
        self.move_to_waypoint('aruco', duration=8.0)
        
        # 5. Apri pinza (rilascio bagaglio)
        self.get_logger().info(f'[Bag {bag_number}] Opening gripper (release)')
        self.control_gripper('open', duration=3.0)
        
        self.get_logger().info(f'--- BAGGAGE {bag_number} COMPLETE ---')
    
    def execute_test_sequence(self):
        """
        Esegue sequenza baggage handling completa
        
        SEQUENZA COMPLETA:
        1. Apri gripper (inizializzazione)
        2. LOOP per ogni bagaglio:
           - Vai a Fra2mo
           - Chiudi pinza
           - Alza bagaglio
           - Vai ad ArUco
           - Apri pinza
        3. Ritorna home (dopo tutti i bagagli)
        """
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING BAGGAGE HANDLING SEQUENCE')
        self.get_logger().info(f'Total bags to handle: {self.num_bags}')
        self.get_logger().info('='*60)
        
        # Inizializzazione: Apri gripper
        self.get_logger().info('[INIT] Opening gripper')
        self.control_gripper('open', duration=3.0)
        
        # LOOP: Gestisci ogni bagaglio
        for bag_num in range(1, self.num_bags + 1):
            self.handle_single_baggage(bag_num)
            
            # Pausa tra bagagli (opzionale)
            if bag_num < self.num_bags:
                self.get_logger().info(f'Preparing for next baggage... (pause 2s)')
                time.sleep(2.0)
        
        # Finale: Ritorno home
        self.get_logger().info('[FINAL] All baggage handled! Returning to HOME')
        self.move_to_waypoint('home', duration=8.0)
        
        self.get_logger().info('='*60)
        self.get_logger().info('SEQUENCE COMPLETE!')
        self.get_logger().info(f'Total bags handled: {self.num_bags}')
        self.get_logger().info('='*60)
        
        # Calcola tempo approssimativo
        time_per_bag = 28  # secondi (8+3+6+8+3)
        total_time = 3 + (time_per_bag * self.num_bags) + 8
        self.get_logger().info(f'Approximate total time: ~{total_time} seconds')
    
    def test_individual_waypoint(self, waypoint_name):
        """Test singolo waypoint (per debug)"""
        self.get_logger().info(f'Testing waypoint: {waypoint_name}')
        self.move_to_waypoint('home', duration=3.0)
        time.sleep(1.0)
        self.move_to_waypoint(waypoint_name, duration=5.0)
        time.sleep(2.0)
        self.move_to_waypoint('home', duration=4.0)


def main(args=None):
    rclpy.init(args=args)
    
    tester = ArmandoWaypointTester()
    
    try:
        # Esegui sequenza completa
        tester.execute_test_sequence()
        
        # Mantieni nodo attivo per permettere osservazione
        tester.get_logger().info('Test complete. Press Ctrl+C to exit.')
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()