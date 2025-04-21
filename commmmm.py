import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime
import threading
import zmq
import time
from codes import StateTopic, CommandTopic

class UnderwaterVehicleGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Underwater Vehicle Control")
        self.root.geometry("800x600")
        self.context = zmq.Context()
        self.connection_active = False
        self._init_zmq()
        
        self.system_states = {}
        self.state_labels = {}
        self._create_gui()
        self._start_receiver_thread()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _init_zmq(self):
        self.command_socket = self.context.socket(zmq.PUB)
        self.state_socket = self.context.socket(zmq.SUB)
        self.state_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        try:
            self.command_socket.bind("tcp://*:8889")
            self.state_socket.connect("tcp://localhost:5555")
        except zmq.ZMQError as e:
            self._log(f"ZMQ Error: {str(e)}", "error")

    def _create_gui(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Status Bar
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=5)
        self.status_label = ttk.Label(status_frame, text="Status: Connected", foreground="green")
        self.status_label.pack(side=tk.LEFT)
        ttk.Button(status_frame, text="Reconnect", command=self._reconnect).pack(side=tk.RIGHT)

        # Notebook
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill=tk.BOTH, expand=True)

        # System States Tab
        states_tab = ttk.Frame(notebook)
        self._create_states_tab(states_tab)
        notebook.add(states_tab, text="System States")

        # Command Interface
        cmd_frame = ttk.Frame(main_frame)
        cmd_frame.pack(fill=tk.X, padx=10, pady=5)
        self.cmd_entry = ttk.Entry(cmd_frame)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(cmd_frame, text="Send", command=self._send_command).pack(side=tk.RIGHT)

        # Log Display
        self.log_area = scrolledtext.ScrolledText(
            main_frame, wrap=tk.WORD, height=8,
            state='disabled', font=('Consolas', 9)
        )
        self.log_area.pack(fill=tk.X, padx=10, pady=(0, 10))
        for tag, color in {'success': 'green', 'error': 'red', 'warning': 'yellow'}.items():
            self.log_area.tag_config(tag, foreground=color)

    def _create_states_tab(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Header
        headers = ["System", "Initialized", "Running", "Controls"]
        for col, text in enumerate(headers):
            ttk.Label(frame, text=text, font=('Arial', 10, 'bold')).grid(row=0, column=col, padx=5, pady=2)

        # System Rows
        systems = {
            0: "Main System",
            1: "Environment System",
            2: "Mission System",
            3: "Motion System",
            4: "Control System"
        }

        for row, (sys_id, sys_name) in enumerate(systems.items(), 1):
            ttk.Label(frame, text=sys_name).grid(row=row, column=0, sticky=tk.W)
            
            # Status Indicators
            init_label = tk.Label(frame, width=10, relief=tk.RIDGE, bg='red')
            init_label.grid(row=row, column=1, padx=2)
            run_label = tk.Label(frame, width=10, relief=tk.RIDGE, bg='red')
            run_label.grid(row=row, column=2, padx=2)
            self.state_labels[sys_id] = (init_label, run_label)

            # Control Buttons
            for col, action in enumerate(['Init', 'Start', 'Stop']):
                ttk.Button(frame, text=action, width=8,
                    command=lambda s=sys_id, a=col: self._send_command(CommandTopic(s, a))
                ).grid(row=row, column=3+col, padx=2)

    def _start_receiver_thread(self):
        self.connection_active = True
        threading.Thread(target=self._receive_messages, daemon=True).start()

    def _receive_messages(self):
        poller = zmq.Poller()
        poller.register(self.state_socket, zmq.POLLIN)
        
        while self.connection_active:
            try:
                if poller.poll(100):
                    msg = self.state_socket.recv_json()
                    self._process_message(msg)
            except zmq.ZMQError as e:
                if e.errno != zmq.ETERM:
                    self._log(f"Receive error: {str(e)}", "error")

    def _process_message(self, msg):
        try:
            state = StateTopic(msg['system'], msg['process'], msg['message'])
            parsed = state.__prs__()
            sys_name = parsed['system']
            process = parsed['process']
            message = parsed['message']

            # Update state indicators
            if process == "Init":
                color = "green" if state.message == 0 else "red"
                self._update_indicator(sys_name, 0, color)
            elif process == "Start":
                color = "green" if state.message == 0 else "yellow"
                self._update_indicator(sys_name, 1, color)

            # Log message
            self._log(f"{sys_name} {process}: {message}", 
                     "success" if state.message == 0 else "error")

        except Exception as e:
            self._log(f"Processing error: {str(e)}", "error")

    def _update_indicator(self, system, index, color):
        for sys_id, name in self.state_labels.items():
            if sys_id == self._get_system_id(system):
                name[index].config(bg=color)
                break

    def _get_system_id(self, system_name):
        system_map = {
            "Main System": 0,
            "Environment System": 1,
            "Mission System": 2,
            "Motion System": 3,
            "Control System": 4
        }
        return system_map.get(system_name, -1)

    def _send_command(self, command=None):
        try:
            if not command:
                parts = list(map(int, self.cmd_entry.get().split(':')))
                command = CommandTopic(parts[0], parts[1])
            
            self.command_socket.send_json(command.__msg__())
            parsed = command.__prs__()
            self._log(f"Sent {parsed['command']} to {parsed['system']}", "info")
            self.cmd_entry.delete(0, tk.END)
        except Exception as e:
            self._log(f"Command error: {str(e)}", "error")

    def _reconnect(self):
        self.connection_active = False
        self.command_socket.close()
        self.state_socket.close()
        self._init_zmq()
        self._start_receiver_thread()
        self._log("Connection restarted", "info")

    def _log(self, message, tag=None):
        self.log_area.config(state=tk.NORMAL)
        self.log_area.insert(tk.END, f"{datetime.now().strftime('%H:%M:%S')} - {message}\n", tag)
        self.log_area.config(state=tk.DISABLED)
        self.log_area.see(tk.END)

    def on_closing(self):
        self.connection_active = False
        self.command_socket.close()
        self.state_socket.close()
        self.context.term()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = UnderwaterVehicleGUI(root)
    root.mainloop()