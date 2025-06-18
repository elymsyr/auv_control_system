# test_subscriber.py
import zmq
import time


context_1 = zmq.Context()
subscriber = context_1.socket(zmq.SUB)
subscriber.connect("tcp://127.0.0.1:7778")
subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

context_2 = zmq.Context()
publisher = context_2.socket(zmq.PUB)
publisher.bind("tcp://127.0.0.1:5560")

print("Listening for transform data on port 5560...")
while True:
    message = subscriber.recv_string()
    publisher.send_string("1.0,2.0,3.0,10.0,20.0,30.0")
    time.sleep(0.1)
    print(f"Received: {message}")