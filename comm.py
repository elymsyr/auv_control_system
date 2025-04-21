import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime
import threading
import zmq
from codes import StateTopic, CommandTopic

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
        self.root.geometry("1000x800")
        self.command_connection_active = False
        self.mission_connection_active = False
        self.environment_connection_active = False
        self.state_connection_active = False
        self.context = zmq.Context()
        
        # ZeroMQ setup
        self.command_port = 8889
        self.data_port = 8888
        self._init_zmq()

        self.create_widgets()
        self._start_receiver_thread()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _init_zmq(self):
        """Initialize ZeroMQ sockets"""
        self.command_socket = self.context.socket(zmq.PUB)
        self.state_socket = self.context.socket(zmq.SUB)  # Renamed from data_socket
        self.state_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        try:
            self.command_socket.bind(f"tcp://*:{self.command_port}")
            # Connect to the common state port used by all systems
            self.state_socket.connect("tcp://localhost:5555")  # Changed from data_port
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
            self.log_message("Invalid mission code. Please enter a valid integer. 0 sent.", 'error')
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
            command=lambda : self.send_command(CommandTopic(system=6, command=self._get_mission_code()+250)),
            width=5
        )
        mission_btn.pack(side=tk.LEFT, padx=0, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Init",
            command=lambda sys=system, act=action: self.send_command(CommandTopic(system=6, command=0))
        )
        btn.grid(row=2, column=col, padx=2, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Start",
            command=lambda sys=system, act=action: self.send_command(CommandTopic(system=6, command=1))
        )
        btn.grid(row=3, column=col, padx=2, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Stop",
            command=lambda sys=system, act=action: self.send_command(CommandTopic(system=6, command=2))
        )
        btn.grid(row=4, column=col, padx=2, pady=2)

        btn = ttk.Button(
            states_frame,
            text="Report",
            command=lambda sys=system, act=action: self.send_command(CommandTopic(system=6, command=3))
        )
        btn.grid(row=5, column=col, padx=2, pady=2)
        
        # -----------------------------------------------

        buttons_frame = ttk.Frame(states_frame)
        buttons_frame.grid(row=row, column=9, rowspan=5, sticky='ew', pady=0, padx=0)
        
        btn = ttk.Button(
            buttons_frame,
            text="Test",
            command=lambda sys=system, act=action: self.send_command()
        )
        btn.grid(row=row, column=col, padx=2, pady=2)
        row += 1

        btn = ttk.Button(
            buttons_frame,
            text="Test",
            command=lambda sys=system, act=action: self.send_command()
        )
        btn.grid(row=row, column=col, padx=2, pady=2)
        row += 1

        btn = ttk.Button(
            buttons_frame,
            text="Test",
            command=lambda sys=system, act=action: self.send_command()
        )
        btn.grid(row=row, column=col, padx=2, pady=2)
        row += 1

        btn = ttk.Button(
            buttons_frame,
            text="Test",
            command=lambda sys=system, act=action: self.send_command()
        )
        btn.grid(row=row, column=col, padx=2, pady=2)
        row += 1

        btn = ttk.Button(
            buttons_frame,
            text="Test",
            command=lambda sys=system, act=action: self.send_command()
        )
        btn.grid(row=row, column=col, padx=2, pady=2)
        row += 1


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

    def send_command(self, command: CommandTopic = None):
        if command is None:
            try:
                code = list(map(int, self.cmd_entry.get().split(':')))
                if len(code) != 2:
                    self.show_error("Invalid command format. Use 'system:command'.")
                    return
                command = CommandTopic(system=int(code[0]), command=int(code[1]))
            except ValueError:
                self.log_message("Invalid command format. Use 'system:command'.")
                return
        try:
            self.command_socket.send_json(command.__msg__())
            self.cmd_entry.delete(0, tk.END)
            self.log_message(f"Sent command: {command.__repr__()}", 'info')
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
            self.command_socket.close()
            self.data_socket.close()
        except zmq.ZMQError as e:
            self.log_message(f"Cleanup error: {str(e)}", 'error')

        self._init_zmq()
        self._start_receiver_thread()
        self.log_message("ZeroMQ connections restarted", 'info')

    def _receive_data(self):
        poller = zmq.Poller()
        poller.register(self.data_socket, zmq.POLLIN)
        
        while self.connection_active:
            try:
                socks = dict(poller.poll(100))
                if self.data_socket in socks:
                    data = self.data_socket.recv_json()
                    self._process_message(data)
            except zmq.ZMQError as e:
                if e.errno != zmq.ETERM:
                    self.log_message(f"Receive error: {str(e)}", 'error')

    def _process_message(self, data):
        msg_type = data.get('type')
        payload = data.get('payload', {})
        
        if msg_type == 'environment':
            self._update_environment(payload)
        elif msg_type == 'system_state':
            self._update_system_state(payload)
        elif msg_type == 'system_log':
            self._update_system_log(payload)

    def _update_system_state(self, states):
        try:
            for system, status in states.items():
                if system in self.system_states:
                    self.system_states[system] = [
                        int(status['initialized']),
                        int(status['running'])
                    ]
            self.update_system_states_gui()
        except Exception as e:
            self.log_message(f"State update error: {str(e)}", 'error')

    def _update_environment(self, data):
        try:
            text = "Environment Status:\n"
            text += f"Position: {data.get('position', [0]*3)}\n"
            text += f"Orientation: {data.get('orientation', [0]*3)}\n"
            text += f"Temperature: {data.get('temperature', 0)}°C\n"
            self._append_to_display(self.env_text, text)
        except Exception as e:
            self.log_message(f"Env update error: {str(e)}", 'error')

    def _update_system_log(self, log):
        try:
            text = "\n".join(
                f"{entry['time']}: {entry['message']}" 
                for entry in log.get('entries', [])
            )
            self._append_to_display(self.history_text, text)
        except Exception as e:
            self.log_message(f"Log update error: {str(e)}", 'error')

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
        self.connection_active = False
        self.command_socket.close()
        self.data_socket.close()
        self.context.term()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = UnderwaterVehicleGUI(root)
    root.mainloop()