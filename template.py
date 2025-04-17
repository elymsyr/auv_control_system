import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime

class BaseGUITemplate:
    def __init__(self, root):
        self.root = root
        self.root.title("Application Template")
        self.root.geometry("1200x800")
        
        # GUI initialization
        self.create_widgets()
        self.create_command_widgets()
        
        # Initialize default state
        self.update_status("Ready", 'green')

    def create_command_widgets(self):
        """Create command input components"""
        command_frame = ttk.Frame(self.root)
        command_frame.pack(fill=tk.X, padx=10, pady=5, anchor=tk.S)

        # Command entry
        self.cmd_entry = ttk.Entry(command_frame, width=50)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.cmd_entry.bind("<Return>", lambda e: self.send_command())

        # Send button
        self.send_btn = ttk.Button(
            command_frame,
            text="Send Command",
            command=self.send_command,
            state=tk.NORMAL
        )
        self.send_btn.pack(side=tk.RIGHT)

        # Command history
        self.cmd_history = scrolledtext.ScrolledText(
            self.root,
            wrap=tk.WORD,
            height=4,
            state='disabled',
            font=('Consolas', 9)
        )
        self.cmd_history.pack(fill=tk.X, padx=10, pady=(0, 10))
        
        # Configure text tags
        self.cmd_history.tag_config('success', foreground='green')
        self.cmd_history.tag_config('error', foreground='red')

    def send_command(self):
        """Handle command sending (template only)"""
        command = self.cmd_entry.get().strip()
        if not command:
            return
            
        self.append_data(self.cmd_history, f"SENT: {command}", 'success')
        self.cmd_entry.delete(0, tk.END)

    def create_widgets(self):
        """Create and arrange GUI components"""
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Status panel
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Status: Ready")
        self.status_label.pack(side=tk.LEFT)
        
        # Action button (placeholder)
        self.action_btn = ttk.Button(
            status_frame, 
            text="Action", 
            command=lambda: self.update_status("Button clicked", 'blue')
        )
        self.action_btn.pack(side=tk.RIGHT)

        # Notebook with tabs
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1 - Data Display
        tab1_frame = ttk.Frame(self.notebook)
        self.tab1_text = scrolledtext.ScrolledText(
            tab1_frame, 
            wrap=tk.WORD, 
            state='disabled',
            font=('Consolas', 10)
        )
        self.tab1_text.pack(fill=tk.BOTH, expand=True)
        self.notebook.add(tab1_frame, text="Data Display")

        # Tab 2 - Messages
        tab2_frame = ttk.Frame(self.notebook)
        self.tab2_text = scrolledtext.ScrolledText(
            tab2_frame, 
            wrap=tk.WORD, 
            state='disabled',
            font=('Consolas', 10)
        )
        self.tab2_text.pack(fill=tk.BOTH, expand=True)
        self.notebook.add(tab2_frame, text="Messages")

        # Configure text tags for all text widgets
        for widget in [self.tab1_text, self.tab2_text]:
            widget.tag_config('info', foreground='blue')
            widget.tag_config('warning', foreground='orange')
            widget.tag_config('error', foreground='red')

    def update_status(self, message, color='black'):
        """Update status label"""
        self.status_label.config(text=message, foreground=color)

    def append_data(self, widget, message, tag=None):
        """Thread-safe text append to specified widget"""
        widget.config(state=tk.NORMAL)
        widget.insert(tk.END, f"{datetime.now().strftime('%H:%M:%S')} - {message}\n", tag)
        widget.config(state=tk.DISABLED)
        widget.see(tk.END)

    def on_closing(self):
        """Cleanup before exit"""
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = BaseGUITemplate(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()