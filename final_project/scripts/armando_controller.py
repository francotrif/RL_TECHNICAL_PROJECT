#!/usr/bin/env python3
"""
Armando Controller - Production Version (FIXED)

✅ FIXED: Proper synchronization of /target_station and /num_bags
Waits for BOTH messages before executing baggage handling.

Usage:
    ros2 run final_project armando_controller.py --ros-args -p station:=1
    
Author: Franco
Date: 2026-01-07
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
import time


class ArmandoController(Node):
    def __init__(self):
        super().__init__('armando_controller')
        
        # Parametro: quale station controllare
        self.declare_parameter('station', 1)
        self.station_id = self.get_parameter('station').value
        
        # Determina colonna (DX o SX)
        self.is_dx_column = (self.station_id % 2 == 1)
        
        # Namespace Armando
        self.namespace = f'armando_{self.station_id}'
        
        # State - ✅ FIXED: Proper synchronization
        self.pending_num_bags = None  # Num bags ricevuto ma non ancora usato
        self.pending_station = False   # Station trigger ricevuto
        self.is_executing = False      # Flag per evitare esecuzioni multiple
        
        # Publishers (comandi ad Armando)
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
        
        # Subscribers (trigger da Fra2mo)
        self.target_sub = self.create_subscription(
            Int32,
            '/target_station',
            self.target_callback,
            10
        )
        
        self.bags_sub = self.create_subscription(
            Int32,
            '/num_bags',
            self.bags_callback,
            10
        )
        
        # Waypoint (testati e ottimizzati)
        if self.is_dx_column:
            # Colonna DX (station 1, 3, 5) - yaw=0, guarda +X
            self.waypoints = {
                'home': [0.0, 0.0, 0.0, 0.0],
                'fra2mo': [0.6, -1.6, -0.0, -0.0],       # Posizione bassa verso Fra2mo
                'lift': [0.6, -1.0, 0.0, 0.0],           # Alza bagaglio
                'aruco': [-0.6, -1.6, -0.0, -0.0],       # Posizione bassa verso ArUco
            }
        else:
            # Colonna SX (station 2, 4, 6) - yaw=π, guarda -X
            self.waypoints = {
                'home': [0.0, 0.0, 0.0, 0.0],
                'fra2mo': [-0.6, -1.6, -0.0, -0.0],      # j0 invertito
                'lift': [-0.6, -1.0, 0.0, 0.0],          # j0 invertito
                'aruco': [0.6, -1.6, -0.0, -0.0],        # j0 invertito
            }
        
        # Gripper states
        self.gripper_open = [0.02, 0.02]
        self.gripper_closed = [0.003, 0.003]
        
        self.get_logger().info(f'Armando {self.station_id} controller initialized')
        self.get_logger().info(f'Column: {"DX" if self.is_dx_column else "SX"}')
        self.get_logger().info(f'Listening on /target_station and /num_bags...')
        
        # Aspetta che publisher siano pronti
        time.sleep(1.0)
    
    def bags_callback(self, msg):
        """
        Callback per topic /num_bags
        Salva il numero di bagagli e controlla se può partire
        """
        self.pending_num_bags = msg.data
        self.get_logger().info(f'Received num_bags: {msg.data}')
        
        # ✅ FIXED: Se station già ricevuta, parti!
        if self.pending_station and not self.is_executing:
            self.execute_with_sync()
    
    def target_callback(self, msg):
        """
        Callback per topic /target_station
        Se station corrisponde, attiva flag e controlla se può partire
        """
        if msg.data == self.station_id:
            if self.is_executing:
                self.get_logger().warn(
                    f'Station {self.station_id} already executing! Ignoring request.'
                )
                return
            
            self.get_logger().info(f'🎯 Station {self.station_id} triggered!')
            self.pending_station = True
            
            # ✅ FIXED: Se num_bags già ricevuto, parti!
            if self.pending_num_bags is not None and not self.is_executing:
                self.execute_with_sync()
    
    def execute_with_sync(self):
        """
        ✅ FIXED: Esegue sequenza SOLO quando ENTRAMBI i messaggi sono arrivati
        """
        if self.pending_station and self.pending_num_bags is not None:
            num_bags = self.pending_num_bags
            
            self.get_logger().info(
                f'🛄​🛩️ Station {self.station_id} starting with {num_bags} bags!'
            )
            
            # Reset flags
            self.pending_station = False
            self.pending_num_bags = None
            
            # Esegui sequenza
            self.is_executing = True
            self.execute_baggage_handling(num_bags)
            self.is_executing = False
    
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
    
    def handle_single_baggage(self, bag_number, total_bags):
        """
        Gestisce un singolo bagaglio (ciclo completo)
        
        SEQUENZA:
        1. Vai a Fra2mo
        2. Chiudi pinza (prelievo)
        3. Alza a posizione intermedia
        4. Vai ad ArUco
        5. Apri pinza (rilascio)
        """
        self.get_logger().info(f'--- 🧳🦾​ HANDLING BAGGAGE {bag_number}/{total_bags} ---')
        
        # 1. Vai a Fra2mo
        self.get_logger().info(f'[Bag {bag_number}] Moving to Fra2mo 🚙 (collect position)')
        self.move_to_waypoint('fra2mo', duration=8.0)
        
        # 2. Chiudi pinza (prelievo bagaglio)
        self.get_logger().info(f'[Bag {bag_number}] Closing gripper 🔧 ​(pickup)')
        self.control_gripper('closed', duration=3.0)
        
        # 3. Alza bagaglio a posizione intermedia sicura
        self.get_logger().info(f'[Bag {bag_number}] Lifting baggage to safe position ⬆️​')
        self.move_to_waypoint('lift', duration=6.0)
        
        # 4. Vai ad ArUco
        self.get_logger().info(f'[Bag {bag_number}] Moving to ArUco 👀​ (release position)')
        self.move_to_waypoint('aruco', duration=8.0)
        
        # 5. Apri pinza (rilascio bagaglio)
        self.get_logger().info(f'[Bag {bag_number}] Opening gripper 🔧 ​(release)')
        self.control_gripper('open', duration=3.0)
        
        self.get_logger().info(f'--- BAGGAGE {bag_number} COMPLETE 🧳🛄  ---')
    
    def execute_baggage_handling(self, num_bags):
        """
        Esegue sequenza baggage handling completa
        
        SEQUENZA:
        1. Apri gripper (inizializzazione)
        2. LOOP per ogni bagaglio:
           - Vai a Fra2mo
           - Chiudi pinza
           - Alza bagaglio
           - Vai ad ArUco
           - Apri pinza
        3. Ritorna home
        """
        self.get_logger().info('='*60)
        self.get_logger().info(f'🦾 STARTING BAGGAGE HANDLING - STATION {self.station_id}')
        self.get_logger().info(f'Total bags to handle: {num_bags}')
        self.get_logger().info('='*60)
        
        # Inizializzazione: Apri gripper
        self.get_logger().info('[INIT] Opening gripper')
        self.control_gripper('open', duration=3.0)
        
        # LOOP: Gestisci ogni bagaglio
        for bag_num in range(1, num_bags + 1):
            self.handle_single_baggage(bag_num, num_bags)
            
            # Pausa tra bagagli
            if bag_num < num_bags:
                self.get_logger().info(f'Preparing for next baggage... (pause 2s)')
                time.sleep(2.0)
        
        # Finale: Ritorno home
        self.get_logger().info('[FINAL] All baggage handled! Returning to HOME 🏠​')
        self.move_to_waypoint('home', duration=8.0)
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'✅🏁​ SEQUENCE COMPLETE - STATION {self.station_id} 🏁​✅')
        self.get_logger().info(f'Total bags handled: {num_bags}')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    controller = ArmandoController()
    
    try:
        # Spin forever (listening mode)
        controller.get_logger().info('Controller active. Waiting for trigger...')
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Controller interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()