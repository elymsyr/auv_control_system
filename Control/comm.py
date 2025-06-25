import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime
import threading
import time
import zmq
from codes import StateTopic, CommandTopic, EnvironmentTopic, MissionTopic

__SYSTEM__ = {
    0: "Main System",
    1: "Environment System",
    2: "Mission System",
    3: "Motion System",
    4: "Control System",
}

class UnderwaterVehicleGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Underwater Vehicle Control")
        self.root.geometry("800x600")
        self.command_connection_active = False
        self.mission_connection_active = False
        self.environment_connection_active = False
        self.state_connection_active = False
        self.context = zmq.Context()

        self.command_socket = None
        self.environment_socket = None
        self.mission_socket = None
        self.state_socket = None

        self.connection_status = {
            'command': False,
            'environment': False,
            'mission': False,
            'state': False
        }
        self.last_received = {
            'environment': 0,
            'mission': 0,
            'state': 0
        }
        
        # ZeroMQ setup
        self.command_port = 8889

        self.create_widgets()
        self._init_zmq()
        self._start_receiver_thread()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _init_zmq(self):
        """Initialize ZeroMQ sockets with proper cleanup"""
        try:
            # Close existing sockets if any
            for sock in [self.command_socket, self.environment_socket, 
                        self.mission_socket, self.state_socket]:
                if sock is not None and not sock.closed:
                    sock.close(linger=0)

            # Create new context
            self.context = zmq.Context()
            
            # Command PUB socket
            self.command_socket = self.context.socket(zmq.PUB)
            self.command_socket.setsockopt(4, 1)
            self.command_socket.bind(f"tcp://*:{self.command_port}")
            self.connection_status['command'] = True
            
            # Environment SUB socket
            self.environment_socket = self.context.socket(zmq.SUB)
            self.environment_socket.connect("tcp://localhost:5560")
            self.environment_socket.setsockopt_string(zmq.SUBSCRIBE, EnvironmentTopic.TOPIC)
            self.connection_status['environment'] = True
            
            # Mission SUB socket
            self.mission_socket = self.context.socket(zmq.SUB)
            self.mission_socket.connect("tcp://localhost:5561")
            self.mission_socket.setsockopt_string(zmq.SUBSCRIBE, MissionTopic.TOPIC)
            self.connection_status['mission'] = True
            
            # State SUB socket
            self.state_socket = self.context.socket(zmq.SUB)
            self.state_socket.connect("tcp://localhost:8888")
            self.state_socket.setsockopt_string(zmq.SUBSCRIBE, StateTopic.TOPIC)
            self.connection_status['state'] = True
            
            self.log_message("ZeroMQ sockets initialized successfully", 'success')
            return True
            
        except zmq.ZMQError as e:
            self.connection_status = {k: False for k in self.connection_status}
            self.show_error(f"ZeroMQ init failed: {str(e)}")
            return False
            
        except zmq.ZMQError as e:
            self.show_error(f"ZeroMQ init failed: {str(e)}")

    def _start_receiver_thread(self):
        """Start data receiving thread"""
        self.connection_active = True
        threading.Thread(target=self._receive_data, daemon=True).start()

    def _get_mission_code(self):
        try:
            return int(self.entry_var.get())
        except ValueError:
            self.log_message("Invalid mission code. Using 0.", 'error')
            return 0

    def create_widgets(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

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

        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        self.env_tab = ttk.Frame(self.notebook)
        self.env_text = scrolledtext.ScrolledText(
            self.env_tab,
            wrap=tk.WORD,
            state='disabled',
            font=('Consolas', 10)
        )
        self.env_text.pack(fill=tk.BOTH, expand=True)
        self.notebook.add(self.env_tab, text="Environment Data")

        self.states_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.states_tab, text="System States")

        # System states grid
        states_frame = ttk.Frame(self.states_tab)
        states_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(states_frame, text="System", font=('Arial', 10, 'bold')).grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        ttk.Label(states_frame, text="Initialized", font=('Arial', 10, 'bold')).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(states_frame, text="Live", font=('Arial', 10, 'bold')).grid(row=0, column=2, padx=5, pady=2)
        ttk.Label(states_frame, text="Actions", font=('Arial', 10, 'bold')).grid(row=0, column=3, columnspan=5, padx=5, pady=2)

        self.state_labels = {}
        row = 1
        for order, system in __SYSTEM__.items():
            ttk.Label(states_frame, text=system).grid(row=row, column=0, padx=5, pady=2, sticky=tk.W)
            
            init_label = tk.Label(states_frame, width=8, relief=tk.RIDGE)
            init_label.grid(row=row, column=1, padx=5, pady=2)
            
            live_label = tk.Label(states_frame, width=8, relief=tk.RIDGE)
            live_label.grid(row=row, column=2, padx=5, pady=2)

            self.state_labels[system] = (init_label, live_label)

            actions = ['init', 'start', 'stop', 'halt', 'reset']
            for col, action in enumerate(actions):
                btn = ttk.Button(
                    states_frame,
                    text=action,
                    command=lambda sys=order, act=col: self.send_command(CommandTopic(system=sys, command=act))
                )
                btn.grid(row=row, column=col+3, padx=2, pady=2)
            
            row += 1

        # Add system data text box at row 6
        self.sys_text = scrolledtext.ScrolledText(
            states_frame,
            wrap=tk.WORD,
            state='disabled',
            font=('Consolas', 10),
            height=10
        )
        self.sys_text.grid(row=row, column=0, rowspan=6, columnspan=9, sticky='nsew', pady=10)

        # Configure grid weights for proper expansion
        states_frame.grid_rowconfigure(6, weight=1)
        for col in range(10):
            states_frame.grid_columnconfigure(col, weight=1)

        col = 9
        self.entry_var = tk.StringVar()
        entry_frame = ttk.Frame(states_frame)
        entry_frame.grid(row=1, column=col, padx=5, pady=2, sticky=tk.W)

        entry = ttk.Entry(entry_frame, textvariable=self.entry_var, width=3)
        entry.pack(side=tk.LEFT, ipady=3, padx=(0, 2), pady=(2, 2))

        mission_btn = ttk.Button(
            entry_frame,
            text="Set",
            command=lambda: self.send_command(CommandTopic(system=6, command=self._get_mission_code())),
            width=5
        )
        mission_btn.pack(side=tk.LEFT, padx=0, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Init",
            command=lambda: self.send_command(CommandTopic(system=6, command=0))
        )
        btn.grid(row=2, column=col, padx=2, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Start",
            command=lambda: self.send_command(CommandTopic(system=6, command=1))
        )
        btn.grid(row=3, column=col, padx=2, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Stop",
            command=lambda: self.send_command(CommandTopic(system=6, command=2))
        )
        btn.grid(row=4, column=col, padx=2, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Report",
            command=lambda: self.send_command(CommandTopic(system=6, command=4))
        )
        btn.grid(row=5, column=col, padx=2, pady=2)

        # Command history
        cmd_frame = ttk.Frame(main_frame)
        cmd_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.cmd_entry = ttk.Entry(cmd_frame)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.cmd_entry.bind("<Return>", lambda e: self.send_command())
        
        self.send_btn = ttk.Button(
            cmd_frame,
            text="Send Command",
            command=self.send_command,
            state=tk.NORMAL
        )
        self.send_btn.pack(side=tk.RIGHT)
        
        self.history_text = scrolledtext.ScrolledText(
            main_frame,
            wrap=tk.WORD,
            height=4,
            state='disabled',
            font=('Consolas', 9)
        )
        self.history_text.pack(fill=tk.X, padx=10, pady=(0, 10))
        
        for widget in [self.env_text, self.history_text]:
            widget.tag_config('success', foreground='green')
            widget.tag_config('error', foreground='red')
            widget.tag_config('warning', foreground='orange')
            widget.tag_config('info', foreground='blue')
        
        self.root.after(1000, self._update_status)


    def send_command(self, command: CommandTopic = None):
        if command is None:
            try:
                code = list(map(int, self.cmd_entry.get().split(':')))
                if len(code) != 2:
                    self.show_error("Invalid command format. Use 'system:command'.")
                    return
                command = CommandTopic(system=int(code[0]), command=int(code[1]))
            except ValueError:
                self.log_message("Invalid command format. Use 'system:command'.", 'error')
                return
        print(f"Sending to {CommandTopic.TOPIC}: {command.__msg__()}")
        try:
            # Send as multi-part message (topic + data)
            self.command_socket.send_string(CommandTopic.TOPIC, zmq.SNDMORE)
            self.command_socket.send(command.serialize())
            self.log_message(f"Sent command: {command.__prs__()}", 'info')
        except Exception as e:
            self.log_message(f"Send failed: {str(e)}", 'error')

    def update_system_states_gui(self):
        for system, (init_label, live_label) in self.state_labels.items():
            state = self.system_states.get(system, [0, 0])
            init_color = 'green' if state[0] == 1 else 'red'
            live_color = 'green' if state[1] == 1 else 'red'
            init_label.config(bg=init_color)
            live_label.config(bg=live_color)

    def restart_server(self):
        """Restart ZeroMQ connections"""
        self.connection_active = False
        try:
            # Close all sockets
            self.command_socket.close(linger=0)
            self.environment_socket.close(linger=0)
            self.mission_socket.close(linger=0)
            self.state_socket.close(linger=0)
            self.context.term()
        except zmq.ZMQError as e:
            self.log_message(f"Cleanup error: {str(e)}", 'error')
        
        # Reset connection timestamps
        for k in self.last_received:
            self.last_received[k] = 0
            
        if self._init_zmq():
            self._start_receiver_thread()
            self.log_message("ZeroMQ connections restarted", 'info')
        else:
            self.log_message("Failed to restart connections", 'error')

    # Update status_var handling
    def _update_status(self):
        status = []
        if self.connection_status['command']:
            status.append("Command: Connected")
        else:
            status.append("Command: Disconnected")
        
        for sys in ['environment', 'mission', 'state']:
            if time.time() - self.last_received[sys] < 5:  # 5-second timeout
                status.append(f"{sys.capitalize()}: Active")
            else:
                status.append(f"{sys.capitalize()}: Inactive")
        
        self.status_var.set(" | ".join(status))
        self.root.after(1000, self._update_status)  # Update every second

    # Modify receive_data to track timestamps
    def _receive_data(self):
        poller = zmq.Poller()
        try:
            poller.register(self.environment_socket, zmq.POLLIN)
            poller.register(self.mission_socket, zmq.POLLIN)
            poller.register(self.state_socket, zmq.POLLIN)
        except zmq.ZMQError as e:
            self.log_message(f"Poller registration failed: {str(e)}", 'error')
            return
        
        while self.connection_active:
            try:
                socks = dict(poller.poll(100))
                for socket in socks:
                    try:
                        topic = socket.recv_string()
                        data = socket.recv()
                        # print(f"Received message on topic: {topic}")
                        self.last_received[topic.lower()] = time.time()
                        self.root.after(0, self._process_message, topic, data)
                    except zmq.ZMQError as e:
                        self.log_message(f"Receive error: {str(e)}", 'error')
            except zmq.ZMQError as e:
                if e.errno != zmq.ETERM:
                    self.log_message(f"Polling error: {str(e)}", 'error')

    def _process_message(self, msg_type, data):
        if msg_type == 'environment':
            self._update_environment(data)
        elif msg_type == 'state':
            self._update_system_state(data)
        elif msg_type == 'mission':
            self._update_mission(data)

    def _update_system_state(self, data):
        try:
            state = StateTopic(**data)
            parsed = state.__prs__()
            system_name = parsed['system']
            if system_name in __SYSTEM__.values():
                initialized = 1 if "Init" in parsed['process'] else 0
                running = 1 if "Live" in parsed['message'] else 0
                self.system_states[system_name] = [initialized, running]
                self.update_system_states_gui()
        except Exception as e:
            self.log_message(f"State update error: {str(e)}", 'error')

    def _update_environment(self, data):
        try:
            text = "Environment Status:\n"
            text += f"Position: {data.get('eta', [0]*6)}\n"
            text += f"Velocity: {data.get('nu', [0]*6)}\n"
            text += f"Acceleration: {data.get('nu_dot', [0]*6)}\n"
            self._append_to_display(self.env_text, text)
        except Exception as e:
            self.log_message(f"Env update error: {str(e)}", 'error')

    def _append_to_display(self, widget, text):
        widget.config(state=tk.NORMAL)
        widget.insert(tk.END, text)
        widget.config(state=tk.DISABLED)
        widget.see(tk.END)

    def log_message(self, message, tag=None):
        self.history_text.config(state=tk.NORMAL)
        self.history_text.insert(tk.END, f"{datetime.now().strftime('%H:%M:%S')} - {message}\n", tag)
        self.history_text.config(state=tk.DISABLED)
        self.history_text.see(tk.END)

    def show_error(self, message):
        messagebox.showerror("Error", message)

    def on_closing(self):
        # Signal all threads to stop
        self.connection_active = False
        
        # Close sockets first
        try:
            self.command_socket.close(linger=0)
            self.environment_socket.close(linger=0)
            self.mission_socket.close(linger=0)
            self.state_socket.close(linger=0)
        except zmq.ZMQError as e:
            pass  # Already closed
        
        # Terminate context
        self.context.term()
        
        # Destroy window
        self.root.destroy()
        
        # Force exit if needed
        import os
        os._exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = UnderwaterVehicleGUI(root)
    root.mainloop()