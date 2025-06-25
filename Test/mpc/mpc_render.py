import zmq
import numpy as np
from render import Render

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")
renderer = Render()

# Receive initial state (binary data) and send acknowledgment.
init_msg = socket.recv()
init_state = np.frombuffer(init_msg, dtype=np.float64)
print("Received initial state:", init_state)
socket.send(b"ACK")

ref = [20, 15, -6]

for _ in range(100):
    # Receive state binary data.
    msg = socket.recv()
    state = np.frombuffer(msg, dtype=np.float64)
    # Process the state as needed (e.g., compute control input)
    # For this example, simply print the received state.
    print("Received state:", state)
    renderer.render(state, ref)
    # Send back a dummy reply to maintain the REQ/REP pattern.
    socket.send(b"OK")
