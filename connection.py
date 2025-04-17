import struct
from datetime import datetime
import zlib
from Crypto.Cipher import AES
import socket

class DataReceiver:
    def __init__(self, key_file):
        with open(key_file, 'rb') as f:
            self.key = f.read(32)
        self.host = '127.0.0.1'
        self.port = 8080

    def parse_data(self, data):
        """Main parsing function returning structured dict"""
        result = {}
        offset = 0
        
        # Environment Data
        env_len = struct.unpack('!I', data[offset:offset+4])[0]
        offset += 4
        result['environment'] = self.parse_environment(data[offset:offset+env_len])
        offset += env_len

        # System Data
        sys_len = struct.unpack('!I', data[offset:offset+4])[0]
        offset += 4
        result['system'] = self.parse_system(data[offset:offset+sys_len])
        
        return result

    def parse_environment(self, data):
        """Parse environment data with type checking"""
        if data[0] != 0x45:
            raise ValueError("Invalid environment header")
        
        offset = 1
        return {
            'current': self.parse_vehicle_state(data, offset),
            'desired': self.parse_vehicle_state_desired(data, offset + 160)
        }

    def parse_vehicle_state(self, data, offset):
        """Parse VehicleState struct"""
        return {
            'eta': struct.unpack_from('>6d', data, offset),
            'nu': struct.unpack_from('>6d', data, offset + 48),
            'nu_dot': struct.unpack_from('>6d', data, offset + 96),
            'temperature': struct.unpack_from('>d', data, offset + 144)[0],
            'pressure': struct.unpack_from('>d', data, offset + 152)[0]
        }

    def parse_vehicle_state_desired(self, data, offset):
        """Parse VehicleStateDes struct"""
        return {
            'eta': struct.unpack_from('>6d', data, offset),
            'nu': struct.unpack_from('>6d', data, offset + 48)
        }

    def parse_system(self, data):
        """Parse system data with validation"""
        if data[0] != 0x4D:
            raise ValueError("Invalid system header")
            
        packs = []
        offset = 1
        while offset + 10 <= len(data):
            packs.append({
                'timestamp': datetime.fromtimestamp(
                    struct.unpack_from('>Q', data, offset)[0]
                ).strftime('%Y-%m-%d %H:%M:%S'),
                'code': struct.unpack_from('>H', data, offset + 8)[0]
            })
            offset += 10
        return {'packs': packs}

    def format_value(self, value):
        """Format numerical values for readability"""
        if isinstance(value, float):
            return round(value, 4)
        if isinstance(value, tuple):
            return tuple(self.format_value(v) for v in value)
        return value

    def print_results(self, parsed):
        """Human-friendly output formatting"""
        print("\n=== Environment Data ===")
        current = parsed['environment']['current']
        desired = parsed['environment']['desired']
        
        print("Current State:")
        print(f"  Position/Orientation (η): {self.format_value(current['eta'])}")
        print(f"  Velocities (ν): {self.format_value(current['nu'])}")
        print(f"  Accelerations (ν_dot): {self.format_value(current['nu_dot'])}")
        print(f"  Temperature: {self.format_value(current['temperature'])}°C")
        print(f"  Pressure: {self.format_value(current['pressure'])} kPa")
        
        print("\nDesired State:")
        print(f"  Target Position/Orientation (η): {self.format_value(desired['eta'])}")
        print(f"  Target Velocities (ν): {self.format_value(desired['nu'])}")
        
        print("\n=== System Data ===")
        for pack in parsed['system']['packs']:
            print(f"Code: 0x{pack['code']:04X} at {pack['timestamp']}")

    def decrypt_and_decompress(self, encrypted_data):
        """Full decryption pipeline"""
        iv = encrypted_data[:12]
        tag = encrypted_data[-16:]
        ciphertext = encrypted_data[12:-16]
        
        cipher = AES.new(self.key, AES.MODE_GCM, nonce=iv)
        compressed = cipher.decrypt_and_verify(ciphertext, tag)
        
        decompressor = zlib.decompressobj(16 + zlib.MAX_WBITS)
        return decompressor.decompress(compressed) + decompressor.flush()

    def start_server(self):
        """Main server loop with error handling"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            print(f"Listening on {self.host}:{self.port}...")

            while True:
                conn, addr = s.accept()
                print(f"Connected by {addr}")
                buffer = bytearray()
                
                try:
                    while True:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buffer.extend(chunk)
                        
                        while len(buffer) >= 28:  # Minimum encrypted message size
                            try:
                                decompressed = self.decrypt_and_decompress(bytes(buffer))
                                parsed = self.parse_data(decompressed)
                                self.print_results(parsed)
                                buffer = bytearray()  # Clear buffer after successful parse
                            except (ValueError, zlib.error, struct.error) as e:
                                print(f"Processing error: {str(e)}")
                                print(f"Buffer dump ({len(buffer)} bytes): {buffer.hex()}")
                                buffer = bytearray()
                                break
                finally:
                    conn.close()
                    print(f"Connection closed by {addr}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python receiver.py <key_file>")
        sys.exit(1)
        
    receiver = DataReceiver(sys.argv[1])
    try:
        receiver.start_server()
    except KeyboardInterrupt:
        print("\nServer shutdown gracefully")