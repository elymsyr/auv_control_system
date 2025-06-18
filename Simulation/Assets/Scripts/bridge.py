import socket
import zmq
import threading
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class UnityBridge:
    def __init__(self):
        self.running = True
        self.transform_connection = None
        self.sensor_connection = None
        
        # ZeroMQ setup
        self.context = zmq.Context()
        
        # For receiving eta/nu data
        self.eta_nu_subscriber = self.context.socket(zmq.SUB)
        self.eta_nu_subscriber.connect(f"tcp://127.0.0.1:5560")
        self.eta_nu_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # For publishing sensor data
        self.sensor_publisher = self.context.socket(zmq.PUB)
        self.sensor_publisher.bind("tcp://*:7778")
        
        # Connection tracking
        self.last_transform_time = 0
        self.last_sensor_time = 0

    def start(self):
        """Start all server threads"""
        threads = [
            threading.Thread(target=self.run_transform_server, daemon=True),
            threading.Thread(target=self.run_sensor_server, daemon=True),
            threading.Thread(target=self.monitor_connections, daemon=True)
        ]
        
        for t in threads:
            t.start()
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.shutdown()

    def run_transform_server(self):
        """TCP server for transform data"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', 7779))
        server.listen(1)
        logging.info("Transform server listening on 127.0.0.1:7779")
        
        while self.running:
            try:
                conn, addr = server.accept()
                logging.info(f"Transform connected: {addr}")
                self.transform_connection = conn
                self.last_transform_time = time.time()
                
                while self.running and self.transform_connection:
                    try:
                        # Get with timeout to allow checking self.running
                        if self.eta_nu_subscriber.poll(100, zmq.POLLIN):
                            data = self.eta_nu_subscriber.recv_string()
                            print(data)
                            conn.sendall(f"{data}\n".encode('utf-8'))
                            self.last_transform_time = time.time()
                    except (socket.error, zmq.ZMQError) as e:
                        logging.warning(f"Transform error: {str(e)}")
                        break
                        
            except socket.timeout:
                continue
            except Exception as e:
                logging.error(f"Transform server error: {str(e)}")
                time.sleep(1)
            finally:
                if self.transform_connection:
                    self.transform_connection.close()
                    self.transform_connection = None
                logging.info("Transform connection closed")
    
    def run_sensor_server(self):
        """TCP server for sensor data"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', 7780))
        server.listen(1)
        logging.info("Sensor server listening on 127.0.0.1:7780")
        
        while self.running:
            try:
                conn, addr = server.accept()
                logging.info(f"Sensor connected: {addr}")
                self.sensor_connection = conn
                self.last_sensor_time = time.time()
                buffer = b''
                
                while self.running and self.sensor_connection:
                    try:
                        conn.settimeout(0.5)
                        data = conn.recv(4096)
                        if not data:
                            break
                            
                        buffer += data
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            self.process_sensor_line(line)
                            self.last_sensor_time = time.time()
                            
                    except socket.timeout:
                        continue
                    except Exception as e:
                        logging.warning(f"Sensor error: {str(e)}")
                        break
                        
            except socket.timeout:
                continue
            except Exception as e:
                logging.error(f"Sensor server error: {str(e)}")
                time.sleep(1)
            finally:
                if self.sensor_connection:
                    self.sensor_connection.close()
                    self.sensor_connection = None
                logging.info("Sensor connection closed")
    
    def process_sensor_line(self, line):
        """Process and publish sensor data"""
        try:
            # Validate data format before publishing
            if b',' in line:
                self.sensor_publisher.send_string(line.decode('utf-8'))
                logging.debug(f"Published sensor data: {line[:64]}...")
            else:
                logging.warning(f"Invalid sensor data: {line[:64]}...")
        except Exception as e:
            logging.error(f"Processing error: {str(e)}")
    
    def monitor_connections(self):
        """Monitor connection health"""
        while self.running:
            time.sleep(5)
            now = time.time()
            
            # Check transform connection
            if self.transform_connection:
                if now - self.last_transform_time > 10:
                    logging.warning("No transform data for 10s. Closing connection.")
                    self.transform_connection.close()
                    self.transform_connection = None
            
            # Check sensor connection
            if self.sensor_connection:
                if now - self.last_sensor_time > 10:
                    logging.warning("No sensor data for 10s. Closing connection.")
                    self.sensor_connection.close()
                    self.sensor_connection = None

    def shutdown(self):
        """Cleanup resources"""
        logging.info("Shutting down...")
        self.running = False
        
        # Close network connections
        for conn in [self.transform_connection, self.sensor_connection]:
            if conn:
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                    conn.close()
                except:
                    pass
        
        # Close ZeroMQ sockets
        self.eta_nu_subscriber.close()
        self.sensor_publisher.close()
        self.context.term()

if __name__ == "__main__":
    bridge = UnityBridge()
    bridge.start()