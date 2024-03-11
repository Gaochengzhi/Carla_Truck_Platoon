import zmq
import logging
from uuid import uuid1


class CommuniAgent:
    def __init__(self, name: str):
        self.context = zmq.Context()
        self.pub_socket = None
        self.sub_sockets = {}
        self.push_socket = None
        self.pull_socket = None
        self.type = name
        self.id = uuid1()

    def init_publisher(self, pub_port):
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{pub_port}")

    def init_subscriber(self, name: str, sub_port, pub_ip="localhost"):
        for existing_name, existing_socket in self.sub_sockets.items():
            if existing_socket.getsockopt(zmq.LAST_ENDPOINT) == f"tcp://{pub_ip}:{sub_port}":
                # Subscriber with the same port already exists, assign the new name
                self.sub_sockets[name] = existing_socket
                return

        sub_socket = self.context.socket(zmq.SUB)
        sub_socket.connect(f"tcp://{pub_ip}:{sub_port}")
        sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub_sockets[name] = sub_socket

    def send_obj(self, data):
        self.pub_socket.send_pyobj(data)

    def send_int(self, data):
        self.pub_socket.send(data)

    def rec_obj(self, sub_name: str):
        try:
            if sub_name in self.sub_sockets:
                msg = self.sub_sockets[sub_name].recv_pyobj(flags=zmq.NOBLOCK)
                return msg
            else:
                raise ValueError(
                    f"No subscriber with name '{sub_name}' initialized.")
        except zmq.Again:
            return None

    def rec_obj_block(self, sub_name: str):
        if sub_name in self.sub_sockets:
            msg = self.sub_sockets[sub_name].recv_pyobj()
            return msg
        else:
            raise ValueError(
                f"No subscriber with name '{sub_name}' initialized.")

    def close(self):
        self.pub_socket.close()
        if self.push_socket:
            self.push_socket.close()
        if self.pull_socket:
            self.pull_socket.close()
        for sub_socket in self.sub_sockets.values():
            sub_socket.close()
        self.context.term()
        exit(0)
