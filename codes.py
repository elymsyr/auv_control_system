import time
import struct

class StateTopic:
    TOPIC = "State"
    def __init__(self, system=0, process=0, message=0):
        self._system = {
            0: "Main System",           # -> _subsystem_process
            1: "Environment System",    # -> _subsystem_process
            2: "Mission System",        # -> _subsystem_process
            3: "Motion System",         # -> _subsystem_process
            4: "Control System",        # -> _subsystem_process
            5: "All Systems",           # -> _subsystem_process
            6: "Mission",               # -> _mission_process
        }

        self._subsystem_process = {
            0: "Init",                  # -> _subsystem_message
            1: "Start",                 # -> _subsystem_message
            2: "Stop",                  # -> _subsystem_message
            3: "Halt",                  # -> _subsystem_message
            4: "Thread",                # -> _subsystem_message
            5: "System",                # -> _subsystem_message
            6: "Other",                 # -> _subsystem_message
        }

        self._mission_process = {
            0: "Test",                  # -> _mission_message
            1: "Mission 1",             # -> _mission_message
            2: "Mission 2",             # -> _mission_message
            3: "Mission 3",             # -> _mission_message
            4: "Mission 4",             # -> _mission_message
            5: "Mission 5",             # -> _mission_message
            6: "Target",                # -> _mission_message
            7: "Line",                  # -> _mission_message
        }

        self._subsystem_message = {
            # General messages (process-independent)
            0: "Operation successful",
            
            # Init process errors
            1: "Sensor initialization failed",
            2: "Calibration timeout",
            
            # Start process errors
            10: "Motor communication failure",
            11: "Power supply unstable",
            
            # ... other error codes
        }

        self._mission_message = {
            0: "Initialized",
            1: "Live",
            2: "Stopped",
            3: "Halted",
        }

        self.system = system & 0b111  # 3-bit unsigned integer (0-7)
        self.process = process & 0b111  # 3-bit unsigned integer (0-7)
        self.message = message & 0xFF  # 8-bit unsigned integer (0-255)
        self.timestamp = int(time.time())  # Current timestamp

    def __repr__(self):
        return (f"StateTopic(system={self.system}, process={self.process}, message={self.message}, timestamp={self.timestamp})")

    def __prs__(self):
        system_name = self._system.get(self.system, "Unknown System")
        if self.system == 7:  # Mission
            process_name = self._mission_process.get(self.process, "Unknown Process")
            message_name = self._mission_message.get(self.message, "Unknown Message")
        else:
            process_name = self._subsystem_process.get(self.process, "Unknown Process")
            message_name = self._subsystem_message.get(self.message, "Unknown Message")

        return {
            "system": system_name,
            "process": process_name,
            "message": message_name,
            "timestamp": self.timestamp,
        }

    def __msg__(self):
        return {
            "system": self.system,
            "message": self.message,
            "process": self.process,
            "timestamp": self.timestamp,
        }

class CommandTopic:
    TOPIC = "Command"
    def __init__(self, system=0, command=0):
        self._system = {
            0: "Main System",           # -> _subsystem_command
            1: "Environment System",    # -> _subsystem_command
            2: "Mission System",        # -> _subsystem_command
            3: "Motion System",         # -> _subsystem_command
            4: "Control System",        # -> _subsystem_command
            5: "All Systems",           # -> _subsystem_command
            6: "Mission",               # -> _mission_command
        }

        self._subsystem_command = {
            0: "Init",
            1: "Start",
            2: "Stop",
            3: "Halt",
            4: "Reset",
        }

        self._mission_command = {
            0: "Set",
            1: "Init",
            2: "Start",
            3: "Stop",
            4: "Report",
        }

        self.system = system & 0b111  # 3-bit unsigned integer (0-7)
        self.command = command & 0xFF  # 8-bit unsigned integer (0-255)
        self.timestamp = int(time.time())  # Current timestamp

    def __repr__(self):
        return (f"CommandTopic(system={self.system}, command={self.command}, timestamp={self.timestamp})")

    def __prs__(self):
        system_name = self._system.get(self.system, "Unknown System")
        if self.system == 7:  # Mission
            command_name = self._mission_command.get(self.command, "Unknown Message")
        else:
            command_name = self._subsystem_command.get(self.command, "Unknown Message")

        return {
            "system": system_name,
            "command": command_name,
            "timestamp": self.timestamp,
        }
    
    def __msg__(self):
        return {
            "system": self.system,
            "command": self.command,
            "timestamp": self.timestamp,
        }
    
    def serialize(self) -> bytes:
        """Pack data into binary format:
        - system: 1 byte (unsigned char)
        - command: 1 byte (unsigned char)
        - timestamp: 8 bytes (unsigned long long)
        """
        return struct.pack('!B B Q', 
                          self.system, 
                          self.command, 
                          self.timestamp)

class EnvironmentTopic:
    TOPIC = "Environment"
    def __init__(self, eta: list[float], nu: list[float], nu_dot: list[float]):        
        self.eta = eta
        self.nu = nu
        self.nu_dot = nu_dot
        self.timestamp = int(time.time())  # Current timestamp

    def __repr__(self):
        return (f"EnvironmentTopic(eta={self.eta}, nu={self.nu}, nu_dot={self.nu_dot}, timestamp={self.timestamp})")

    def __prs__(self):
        return {
            "eta": self.eta,
            "nu": self.nu,
            "nu_dot": self.nu_dot,
            "timestamp": self.timestamp,
        }

    def __msg__(self):
        return {
            "eta": self.eta,
            "nu": self.nu,
            "nu_dot": self.nu_dot,
            "timestamp": self.timestamp,
        }

class MissionTopic:
    TOPIC = "Mission"
    def __init__(self, eta_des: list[float], nu_des: list[float], nu_dot_des: list[float]):
        self.eta_des = eta_des
        self.nu_des = nu_des
        self.nu_dot_des = nu_dot_des
        self.timestamp = int(time.time())  # Current timestamp

    def __repr__(self):
        return (f"MissionTopic:(eta_des={self.eta_des}, nu_des={self.nu_des}, nu_dot_des={self.nu_dot_des}, timestamp={self.timestamp})")

    def __prs__(self):
        return {
            "eta_des": self.eta_des,
            "nu_des": self.nu_des,
            "nu_dot_des": self.nu_dot_des,
            "timestamp": self.timestamp,
        }

    def __msg__(self):
        return {
            "eta_des": self.eta_des,
            "nu_des": self.nu_des,
            "nu_dot_des": self.nu_dot_des,
            "timestamp": self.timestamp,
        }
