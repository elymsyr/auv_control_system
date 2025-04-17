import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime
import os
import threading
import socket
import zlib
from Crypto.Cipher import AES
import struct
import traceback

ORDER = {
    0: "ALL",
    1: "CommunicationSystem",
    2: "SensorSystem",
    3: "EnvironmentSystem",
    4: "MotionSystem",
    5: "ControlSystem",
}

MODE = {
    0: "initialized",
    1: "halted",
    2: "suspended",
    3: "resumed",
    4: "restarted",
}

SUCCESS = {
    0: "Fail",
    1: "Success",
    2: "Already",
}

def handleMessage(number, success, mode, order):
    if number == 0:
        return f"{success} {mode} {order}"

class UnderwaterVehicleGUI:
    def __init__(self, root, key_file="/home/eren/GitHub/underwater/UnderwaterVehicleControl/encryption_keys.bin"):
        self.root = root
        self.root.title("Underwater Vehicle Control")
        self.root.geometry("800x600")
        self.key_file = key_file
        self.connection_active = False
        self.sock = None
        self.client_socket = None
        self.notification_flags = {"environment": False, "system": False}
        self.host = '127.0.0.1'
        self.port = 8080

        self.MESSAGES = {
            (2, 2): {
                (2, 2): "Shutting down...",
            },
            (1, 9): {
                (0, 0): "Health check failed",
                (0, 1): "Loop detached",
                (0, 2): "Resetting",
                (0, 3): "Detached",
                (0, 4): "",
                (0, 5): "",
                (0, 6): "",
                (0, 7): "",
                (0, 8): "",
                (0, 9): "",
            },
        }

        self.system_states = {
            "SensorSystem": [0, 0],
            "EnvironmentSystem": [0, 0],
            "MotionSystem": [0, 0],
            "ControlSystem": [0, 0],
            "CommunicationSystem": [0, 0],
        }
        
        self.create_widgets()
        self.setup_communication()
        self.start_server()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Status bar
        self.status_frame = ttk.Frame(main_frame)
        self.status_frame.pack(fill=tk.X, pady=5)
        
        self.status_var = tk.StringVar(value="Status: Waiting for connection")
        self.status_label = ttk.Label(
            self.status_frame, 
            textvariable=self.status_var,
            foreground='orange'
        )
        self.status_label.pack(side=tk.LEFT)
        
        self.reconnect_btn = ttk.Button(
            self.status_frame, 
            text="Restart Server", 
            command=self.restart_server,
            state=tk.NORMAL
        )
        self.reconnect_btn.pack(side=tk.RIGHT)

        # Notebook (Tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Environment Data Tab
        self.env_tab = ttk.Frame(self.notebook)
        self.env_text = scrolledtext.ScrolledText(
            self.env_tab,
            wrap=tk.WORD,
            state='disabled',
            font=('Consolas', 10)
        )
        self.env_text.pack(fill=tk.BOTH, expand=True)
        self.notebook.add(self.env_tab, text="Environment Data")
        
        # System Data Tab
        self.sys_tab = ttk.Frame(self.notebook)
        self.sys_text = scrolledtext.ScrolledText(
            self.sys_tab, 
            wrap=tk.WORD, 
            state='disabled',
            font=('Consolas', 10)
        )
        self.sys_text.pack(fill=tk.BOTH, expand=True)
        self.notebook.add(self.sys_tab, text="System Data")

        # System States Tab
        self.states_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.states_tab, text="System States")

        # System States Visualization
        self.state_labels = {}
        states_frame = ttk.Frame(self.states_tab)
        states_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Header
        ttk.Label(states_frame, text="System", font=('Arial', 10, 'bold')).grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        ttk.Label(states_frame, text="Initialized", font=('Arial', 10, 'bold')).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(states_frame, text="Live", font=('Arial', 10, 'bold')).grid(row=0, column=2, padx=5, pady=2)

        # Create state indicators for each system
        row = 1
        for system in self.system_states:
            ttk.Label(states_frame, text=system).grid(row=row, column=0, padx=5, pady=2, sticky=tk.W)
            
            # Initialized indicator
            init_label = tk.Label(states_frame, width=8, relief=tk.RIDGE)
            init_label.grid(row=row, column=1, padx=5, pady=2)
            
            # Live indicator
            live_label = tk.Label(states_frame, width=8, relief=tk.RIDGE)
            live_label.grid(row=row, column=2, padx=5, pady=2)
            
            self.state_labels[system] = (init_label, live_label)
            row += 1

        # Command Section
        cmd_frame = ttk.Frame(main_frame)
        cmd_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.cmd_entry = ttk.Entry(cmd_frame, width=50)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.cmd_entry.bind("<Return>", lambda e: self.send_command())
        
        self.send_btn = ttk.Button(
            cmd_frame,
            text="Send Command",
            command=self.send_command,
            state=tk.DISABLED
        )
        self.send_btn.pack(side=tk.RIGHT)
        
        # History
        self.history_text = scrolledtext.ScrolledText(
            main_frame,
            wrap=tk.WORD,
            height=4,
            state='disabled',
            font=('Consolas', 9)
        )
        self.history_text.pack(fill=tk.X, padx=10, pady=(0, 10))
        
        # Configure tags
        for widget in [self.env_text, self.sys_text, self.history_text]:
            widget.tag_config('success', foreground='green')
            widget.tag_config('error', foreground='red')
            widget.tag_config('warning', foreground='orange')
            widget.tag_config('info', foreground='blue')

    def update_system_states_gui(self):
        for system, (init_label, live_label) in self.state_labels.items():
            state = self.system_states.get(system, [0, 0])
            init_color = 'green' if state[0] == 1 else 'red'
            live_color = 'green' if state[1] == 1 else 'red'
            init_label.config(bg=init_color)
            live_label.config(bg=live_color)

    def setup_communication(self):
        try:
            with open(self.key_file, 'rb') as f:
                self.aes_key = f.read(32)
            self.log_message("Encryption initialized successfully")
        except Exception as e:
            self.show_error(f"Failed to initialize encryption: {str(e)}")
            self.root.destroy()

    def start_server(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.listen(1)
            self.sock.settimeout(1)
            self.update_status("Waiting for connection...", 'orange')
            self.log_message(f"Server started on {self.host}:{self.port}")
            threading.Thread(target=self.accept_connections, daemon=True).start()
        except Exception as e:
            self.show_error(f"Failed to start server: {str(e)}")
            self.update_status("Server failed", 'red')

    def accept_connections(self):
        while hasattr(self.root, '_windowingsystem'):  # Check if window exists
            try:
                if not self.connection_active:
                    self.client_socket, addr = self.sock.accept()
                    self.client_socket.settimeout(1)
                    self.connection_active = True
                    self.update_status(f"Connected: {addr[0]}", 'green')
                    self.send_btn.config(state=tk.NORMAL)
                    self.log_message(f"Connection established with {addr}")
                    threading.Thread(target=self.receive_data, daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if hasattr(self.root, '_windowingsystem'):
                    self.log_message(f"Connection error: {str(e)}", 'error')
                break

    def restart_server(self):
        """Restart the server connection"""
        self.update_status("Restarting server...", 'orange')
        self.send_btn.config(state=tk.DISABLED)
        self.connection_active = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        
        self.start_server()

    def receive_data(self):
        buffer = bytearray()
        while self.connection_active:
            try:
                data = self.client_socket.recv(4096)
                if not data:
                    raise ConnectionError("Connection closed by client")
                buffer.extend(data)
                
                while True:
                    processed_bytes = self.process_message(buffer)
                    if processed_bytes == 0:
                        break
                    buffer = buffer[processed_bytes:]
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.handle_connection_error(str(e))
                break

    def process_message(self, data):
        try:
            if len(data) < 28:
                return 0  # Not enough data for IV + min ciphertext + tag

            iv = data[:12]
            tag = data[-16:]
            ciphertext = data[12:-16]
            
            cipher = AES.new(self.aes_key, AES.MODE_GCM, nonce=iv)
            compressed = cipher.decrypt_and_verify(ciphertext, tag)
            
            decompressor = zlib.decompressobj(16 + zlib.MAX_WBITS)
            payload = decompressor.decompress(compressed) + decompressor.flush()
            
            parsed = self.parse_data(payload)
            self.root.after(0, self.update_environment, parsed['environment'])
            self.root.after(0, self.update_system, parsed['system'])
            self.root.after(0, self.update_system_states_gui)  # Update state indicators
            
            return len(data)  # Full message processed
        except (ValueError, zlib.error, struct.error, AES.InvalidTag) as e:
            self.log_message(f"Processing error: {str(e)}", 'error')
            return len(data)  # Clear buffer on error
        except Exception as e:
            self.log_message(f"Unexpected error: {str(e)}", 'error')
            return len(data)

    def handle_connection_error(self, error_msg):
        """Handle connection errors gracefully"""
        self.connection_active = False
        self.update_status("Connection lost", 'red')
        self.send_btn.config(state=tk.DISABLED)
        self.log_message(f"Connection error: {error_msg}", 'error')
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass

    def parse_data(self, data):
        result = {}
        offset = 0
        
        try:
            # Environment data section
            if len(data) - offset < 4:
                return result
            env_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            
            if env_len > 0:
                result['environment'] = self.parse_environment(data[offset:offset+env_len])
                offset += env_len

            # System data section
            if len(data) - offset < 4:
                return result
            sys_len = struct.unpack('!I', data[offset:offset+4])[0]
            offset += 4
            
            if sys_len > 0:
                result['system'] = self.parse_system(data[offset:offset+sys_len])
            
            return result
        except Exception as e:
            self.log_message(f"Parse error: {str(e)}", 'error')
            return result

    def parse_environment(self, data):
        try:
            if data[0] != 0x45:
                raise ValueError("Invalid environment header")
            
            offset = 1
            return {
                'current': self.parse_vehicle_state(data, offset),
                'desired': self.parse_vehicle_state_desired(data, offset + 160)
            }
        except Exception as e:
            self.log_message(f"Environment parse error: {str(e)}", 'error')
            return {'current': {}, 'desired': {}}

    def parse_vehicle_state(self, data, offset):
        return {
            'eta': struct.unpack_from('>6d', data, offset),
            'nu': struct.unpack_from('>6d', data, offset + 48),
            'nu_dot': struct.unpack_from('>6d', data, offset + 96),
            'temperature': struct.unpack_from('>d', data, offset + 144)[0],
            'pressure': struct.unpack_from('>d', data, offset + 152)[0]
        }

    def parse_vehicle_state_desired(self, data, offset):
        return {
            'eta': struct.unpack_from('>6d', data, offset),
            'nu': struct.unpack_from('>6d', data, offset + 48)
        }

    def parse_system(self, data):
        try:
            if data[0] != 0x4D:
                raise ValueError("Invalid system header")
            
            packs = []
            offset = 1
            data_len = len(data)
            
            while offset + 10 <= data_len:
                # Parse timestamp (8 bytes) and code (2 bytes)
                timestamp = struct.unpack_from('>Q', data, offset)[0]
                code = struct.unpack_from('>H', data, offset + 8)[0]
                offset += 10  # Always advance offset by packet size

                timestamp_str = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')

                message_text = self.parse_system_message(code=int(code))
                if message_text is not None:
                    packs.append({
                        'timestamp': timestamp_str,
                        'code': message_text
                    })

            return {'packs': packs}
        except Exception as e:
            error_details = traceback.format_exc()
            self.log_message(f"System parse error: {str(e)}\n{error_details}", 'error')
            return {'packs': []}

    def parse_system_message(self, code: int):
        message_text = None
        
        system = (code // 10000) % 10
        number = (code // 1000) % 10
        success = (code // 100) % 10
        mode = (code // 10) % 10
        order = code % 10
        order_text = ORDER.get(order, 'Unknown')
        
        statement = (system, number)
        mode_couple = (success, mode)
        
        message_found = self.MESSAGES.get(statement, None)
        if message_found is not None:
            text = message_found.get(mode_couple, "Unknown message")
            message_text = f"{order_text} - {message_text}"
        elif statement == (2, 0):
            self.system_states[order_text] = mode_couple
        elif statement == (1, 0):
            message_text = f"{order_text} - Restarting ..."
        elif statement == (0, 0):
            message_text = f"{order_text} - {SUCCESS.get(success, 'Unknown')} {MODE.get(mode, 'Unknown')}"
        
        return message_text

    def format_value(self, value):
        if isinstance(value, float):
            return round(value, 4)
        if isinstance(value, (tuple, list)):
            return tuple(self.format_value(v) for v in value)
        return value

    def update_environment(self, env_data):
        try:
            current = env_data.get('current', {})
            desired = env_data.get('desired', {})
            
            formatted = " Current State:\n"
            formatted += f"  Position/Orientation (η): {self.format_value(current.get('eta', (0,)*6))}\n"
            formatted += f"  Velocities (ν): {self.format_value(current.get('nu', (0,)*6))}\n"
            formatted += f"  Accelerations (ν_dot): {self.format_value(current.get('nu_dot', (0,)*6))}\n"
            formatted += f"  Temperature: {self.format_value(current.get('temperature', 0))}°C\n"
            formatted += f"  Pressure: {self.format_value(current.get('pressure', 0))} kPa\n\n"
            
            formatted += "Desired State:\n"
            formatted += f"  Target Position/Orientation (η): {self.format_value(desired.get('eta', (0,)*6))}\n"
            formatted += f"  Target Velocities (ν): {self.format_value(desired.get('nu', (0,)*6))}\n"
            
            self.append_data(self.env_text, formatted, add_timestamp=False)
            if not self.notification_flags['environment']:
                self.notebook.tab(0, text="Environment Data *")
                self.notification_flags['environment'] = True
        except Exception as e:
            self.log_message(f"Environment update error: {str(e)}", 'error')

    def update_system(self, sys_data):
        try:
            formatted = ""
            for pack in sys_data.get('packs', []):
                formatted += f"{pack.get('timestamp', '')}: {pack.get('code', '')}\n"
            
            if formatted:
                self.append_data(self.sys_text, formatted, add_timestamp=False)
                if not self.notification_flags['system']:
                    self.notebook.tab(1, text="System Data *")
                    self.notification_flags['system'] = True
        except Exception as e:
            self.log_message(f"System update error: {str(e)}", 'error')

    def format_data(self, data):
        """Convert binary data to readable format"""
        try:
            text = data.decode('utf-8', errors='replace')
            return f"Hex: {data.hex()}\nText: {text}"
        except Exception as e:
            return f"Data format error: {str(e)}"

    def send_command(self):
        try:
            integer = int(self.cmd_entry.get().strip())
            if not 0 <= integer <= 24999:
                raise ValueError("Integer must be 0-24999")

            packed = struct.pack('!i', integer)

            compressor = zlib.compressobj(level=zlib.Z_BEST_SPEED, wbits=-15)
            compressed = compressor.compress(packed) + compressor.flush()

            iv = os.urandom(12)
            cipher = AES.new(self.aes_key, AES.MODE_GCM, nonce=iv)
            ciphertext, tag = cipher.encrypt_and_digest(compressed)
            
            # Message structure: [length][12-byte IV][ciphertext][16-byte tag]
            message = iv + ciphertext + tag
            self.client_socket.sendall(struct.pack('!I', len(message)) + message)
        except ValueError as ve:
            self.log_message(f"Invalid input: {ve}", 'error')
        except Exception as e:
            self.log_message(f"Send failed: {str(e)}", 'error')
        except Exception as e:
            print(f"Python send error: {str(e)}")

    def update_status(self, message, color='black'):
        """Update status label"""
        self.status_var.set(f"Status: {message}")
        self.status_label.config(foreground=color)

    def append_data(self, widget, message, tag=None, add_timestamp=True):
        """Thread-safe text append with optional timestamp"""
        widget.config(state=tk.NORMAL)
        if add_timestamp:
            timestamp = f"{datetime.now().strftime('%H:%M:%S')} - "
        else:
            timestamp = ""
        widget.insert(tk.END, f"{timestamp}{message}\n", tag)
        widget.config(state=tk.DISABLED)
        widget.see(tk.END)

    def log_message(self, message, tag=None):
        """Add message to history"""
        self.append_data(self.history_text, message, tag)

    def show_error(self, message):
        """Show error dialog"""
        messagebox.showerror("Error", message)

    def on_closing(self):
        """Handle window closing"""
        self.connection_active = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = UnderwaterVehicleGUI(root)
    root.mainloop()