import zmq
import struct
import math
import time
import random
import threading
import logging
import socket  # Fixed import
import queue   # Fixed import for Queue

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Global flag for shutdown
running = True

# Shared data structures
env_lock = threading.Lock()
sonar_queue = queue.Queue()  # Fixed queue reference
latest_environment = {
    'position': [0.0, 0.0, 0.0],
    'rotation': [0.0, 0.0, 0.0]
}

def calculate_detection_angles(degree, num_detections, fov):
    # Calculate angular resolution
    angular_resolution = fov / (num_detections - 1)
    
    # Calculate starting angle (leftmost detection)
    start_angle = degree - (fov / 2.0)
    
    # Calculate angle for each detection and normalize to 0-360
    detection_angles = [(start_angle + i * angular_resolution) % 360 for i in range(num_detections)]
    return detection_angles

class EnvironmentSubscriber(threading.Thread):
    """ZeroMQ Subscriber for C++ EnvironmentPublisher"""
    def __init__(self, endpoint):
        super().__init__()
        self.endpoint = endpoint
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
        self.daemon = True
        
        try:
            self.socket.connect(self.endpoint)
            self.socket.setsockopt_string(zmq.SUBSCRIBE, "Environment")
            logging.info(f"Subscribed to Environment topic at {self.endpoint}")
        except zmq.ZMQError as e:
            logging.error(f"Connection failed: {str(e)}")

    def run(self):
        while running:
            try:
                # Receive multipart message
                message = self.socket.recv_multipart()
                
                if len(message) == 2 and message[0] == b"Environment":
                    self.process_message(message[1])
                else:
                    logging.warning(f"Unexpected message format: {len(message)} parts")
            except zmq.Again:
                pass  # Timeout, check running status
            except Exception as e:
                logging.error(f"EnvironmentSubscriber error: {str(e)}")
                time.sleep(1)

    def process_message(self, data):
        # Verify expected length (18 doubles = 144 bytes)
        if len(data) != 144:
            logging.warning(f"Invalid environment data length: {len(data)} bytes")
            return
            
        try:
            # Unpack 18 doubles
            values = struct.unpack('18d', data)
            
            # Extract position and orientation (first 6 values)
            # eta[0:3] = position, eta[3:6] = orientation (radians)
            pos_x, pos_y, pos_z = values[0], values[1], values[2]
            roll, pitch, yaw = values[3], values[4], values[5]
            
            # Convert radians to degrees
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # Update shared environment data
            with env_lock:
                latest_environment['position'] = [pos_x, pos_y, pos_z]
                latest_environment['rotation'] = [roll_deg, pitch_deg, yaw_deg]
                
            logging.debug(f"Updated environment: pos=({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f}) "
                          f"rot=({roll_deg:.2f}, {pitch_deg:.2f}, {yaw_deg:.2f})")

        except Exception as e:
            logging.error(f"Environment data processing error: {str(e)}")

    def stop(self):
        self.socket.close()
        self.context.term()

class TestSonarPublisher(threading.Thread):
    """ZeroMQ Publisher for C++ TestSonarSubscriber"""
    def __init__(self, endpoint):
        super().__init__()
        self.endpoint = endpoint
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.daemon = True
        
        try:
            self.socket.bind(self.endpoint)
            logging.info(f"TestSonarPublisher bound to {self.endpoint}")
            # Allow publisher to initialize
            time.sleep(0.2)
        except zmq.ZMQError as e:
            logging.error(f"Bind failed: {str(e)}")

    def run(self):
        while running:
            try:
                # Get sonar data from queue (blocking with timeout)
                sonar_data = sonar_queue.get(timeout=0.1)
                
                # Pack data: 10 detection values + degree + timestamp
                detection = sonar_data.get('detection', [0]*10)
                degree = sonar_data.get('degree', 0.0)
                degree_cal = calculate_detection_angles(degree, 10, degree)
                timestamp = sonar_data.get('timestamp', time.time())
                
                data = struct.pack('10d 10d d', *detection, *degree_cal, timestamp)
                self.socket.send_multipart([b"TestSonar", data])
                
                logging.debug(f"Published sonar data: {len(detection)} points")
            except queue.Empty:  # Fixed queue reference
                pass  # No data, continue
            except Exception as e:
                logging.error(f"Sonar publish error: {str(e)}")
                time.sleep(1)

    def stop(self):
        self.socket.close()
        self.context.term()

class UnityEnvironmentServer(threading.Thread):
    """TCP Server for Unity's EnvironmentTcpSubscriber"""
    def __init__(self, host='127.0.0.1', port=7779):
        super().__init__()
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Fixed socket reference
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(5)
        self.clients = []
        self.daemon = True
        logging.info(f"Unity Environment Server started on {host}:{port}")

    def run(self):
        while running:
            try:
                client_socket, addr = self.server_socket.accept()
                logging.info(f"Unity Environment client connected: {addr}")
                self.clients.append(client_socket)
                threading.Thread(
                    target=self.handle_client, 
                    args=(client_socket,),
                    daemon=True
                ).start()
            except socket.error as e:  # Fixed socket reference
                if running:
                    logging.error(f"Accept error: {str(e)}")
                break

    def handle_client(self, client_socket):
        try:
            while running:
                with env_lock:
                    pos = latest_environment['position']
                    rot = latest_environment['rotation']
                
                # Format as comma-separated string
                data_str = (f"{pos[1]:.4f},{(pos[2]):.4f},{pos[0]:.4f},"
                            f"{rot[1]:.4f},{rot[2]:.4f},{rot[0]:.4f}")
                data_bytes = data_str.encode('utf-8')
                
                # Add little-endian length header
                header = struct.pack('>I', len(data_bytes))
                client_socket.sendall(header + data_bytes)
                time.sleep(0.05)  # ~20Hz
        except (ConnectionResetError, BrokenPipeError):
            pass  # Client disconnected
        except Exception as e:
            logging.error(f"Environment client error: {str(e)}")
        finally:
            client_socket.close()
            if client_socket in self.clients:
                self.clients.remove(client_socket)
            logging.info("Unity Environment client disconnected")

    def stop(self):
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        self.server_socket.close()

class UnitySonarServer(threading.Thread):
    """TCP Server for Unity's SonarTcpPublisher"""
    def __init__(self, host='127.0.0.1', port=7780):
        super().__init__()
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Fixed socket reference
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(5)
        self.clients = []
        self.daemon = True
        logging.info(f"Unity Sonar Server started on {host}:{port}")

    def run(self):
        while running:
            try:
                client_socket, addr = self.server_socket.accept()
                logging.info(f"Unity Sonar client connected: {addr}")
                self.clients.append(client_socket)
                threading.Thread(
                    target=self.handle_client, 
                    args=(client_socket,),
                    daemon=True
                ).start()
            except socket.error as e:  # Fixed socket reference
                if running:
                    logging.error(f"Accept error: {str(e)}")
                break

    def handle_client(self, client_socket):
        try:
            while running:
                # Read 4-byte big-endian length header
                header = client_socket.recv(4)
                if not header:
                    break  # Connection closed

                # Unpack message length
                msg_length = struct.unpack('>I', header)[0]
                
                # Validate message length
                if msg_length > 1024 * 1024:  # 1MB sanity check
                    logging.error(f"Invalid sonar message length: {msg_length}")
                    break
                # Read message data
                data_bytes = b''
                while len(data_bytes) < msg_length:
                    chunk = client_socket.recv(min(4096, msg_length - len(data_bytes)))
                    if not chunk:
                        break
                    data_bytes += chunk
                
                if len(data_bytes) != msg_length:
                    logging.warning(f"Incomplete sonar message: {len(data_bytes)}/{msg_length}")
                    continue
                
                # Process sonar data
                data_str = data_bytes.decode('utf-8')
                sonar_points = [float(x) for x in data_str.split(',')]
                
                # Prepare data for ZeroMQ publisher
                sonar_data = {
                    'detection': sonar_points[:10],  # First 10 points
                    'degree': 10.0,  # Default value
                    'timestamp': time.time()
                }
                
                # Add to queue for ZeroMQ publisher
                sonar_queue.put(sonar_data)
                logging.debug(f"Received sonar data: {len(sonar_points)} points")
                
        except (ConnectionResetError, BrokenPipeError):
            pass  # Client disconnected
        except Exception as e:
            logging.error(f"Sonar client error: {str(e)}")
        finally:
            client_socket.close()
            if client_socket in self.clients:
                self.clients.remove(client_socket)
            logging.info("Unity Sonar client disconnected")

    def stop(self):
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        self.server_socket.close()

def main():
    global running
    
    # Configuration
    ZMQ_ENV_ENDPOINT = "tcp://localhost:5560"   # C++ EnvironmentPublisher endpoint
    ZMQ_SONAR_ENDPOINT = "tcp://*:7778"          # Python binds this for C++ to connect
    UNITY_ENV_PORT = 7779
    UNITY_SONAR_PORT = 7780
    
    # Create components
    env_sub = EnvironmentSubscriber(ZMQ_ENV_ENDPOINT)
    sonar_pub = TestSonarPublisher(ZMQ_SONAR_ENDPOINT)
    unity_env_server = UnityEnvironmentServer(host="192.168.229.107", port=UNITY_ENV_PORT)
    unity_sonar_server = UnitySonarServer(host="192.168.229.107", port=UNITY_SONAR_PORT)
    
    # Start components
    env_sub.start()
    sonar_pub.start()
    unity_env_server.start()
    unity_sonar_server.start()
    
    logging.info("Bridge is running. Press Ctrl+C to exit.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
        running = False
        
        # Stop components
        env_sub.stop()
        sonar_pub.stop()
        unity_env_server.stop()
        unity_sonar_server.stop()
        
        # Wait for threads to finish
        env_sub.join(timeout=1.0)
        sonar_pub.join(timeout=1.0)
        unity_env_server.join(timeout=1.0)
        unity_sonar_server.join(timeout=1.0)
        
        logging.info("Clean shutdown complete")

if __name__ == "__main__":
    main()