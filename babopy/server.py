import json
import socket
import typing
import random

from .const import *
from .utils import algo_map
from .algorithms import BaseAlgorithm

class Server:
    """
    The server class is used to communicate with the client.
    This is the backbone of babopy and implemented to handle everything regarding the simulation on server side.
    """
    def __init__(self, host, port, algorithm):
        """
        Initializes the server.
        :param host: The host address to bind to
        :type host: str
        :param port: The port to bind to
        :type port: int
        :param algorithm: The algorithm to use
        :type algorithm: str
        """
        self.port: int = port
        self.host: str = host
        if isinstance(algorithm, str):
            self.action_space: BaseAlgorithm = algo_map.get(algorithm, algo_map['random'])()
        else:
            self.action_space: BaseAlgorithm = algorithm

        self.last_update: typing.Tuple[float, float, bool, bool, dict] = (0.0, 0.0, False, False, {})
        self.socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


    def start(self) -> None:
        """
        Starts the server. This is a blocking call. This method will not return until the client has connected.
        For using it as a part of a larger program, it is recommended to run it in a separate thread.
        :return: None
        """
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        self.conn, self.addr = self.socket.accept()
    
    def send(self, data: str) -> None:
        """
        This is a helper function. Takes a string as an argument and encodes it to bytes before sending to client.
        :param data: The data to send
        :type data: str
        :return: None
        """
        if not isinstance(data, str):
            raise TypeError('data must be a string')
        self.conn.send(data.encode())

    def receive(self, size: int=1024) -> str:
        """
        This is a helper function. Receives data from client and decodes it to string before returning.
        """
        return self.conn.recv(size).decode()

    def fetch_action_space(self) -> int:
        """
        Fetches the action space from the client.
        Uses the 'ACTSPACE' command as per the protocol.
        :return: The action space
        :rtype: int
        """
        self.send(ACTSPACE)
        data = self.receive()
        try:
            action_space = int(data)
            self.action_space.update_action_space(action_space)
            return action_space
        except ValueError:
            return -1

    def send_action(self, action) -> None:
        """
        Sends an action to the client.
        Uses the 'ACTION <action>' command as per the protocol.
        :param action: The action to send
        :type action: int
        :return: None
        """
        self.send(f'{ACTION} {action}')

    def fetch_results(self) -> typing.Tuple[float, float, bool, bool, dict]:
        """
        Fetches the action result from the client. The data is in JSON format.
        Keys are 'observation', 'reward', 'termination', 'truncation' and 'info'.
        :return: The action result
        :rtype: tuple
        """
        _data = self.receive()
        try:
            data = json.loads(_data)
            self.last_update = (data['observation'], data['reward'], data['terminated'], data['truncated'], data['info'])
            return self.last_update
        except json.decoder.JSONDecodeError:
            return (0.0, 0.0, False, False, {})

    def get_action(self) -> int:
        """
        Returns the next action based on the action space.
        :return: The next action
        :rtype: int
        """
        return self.action_space.get_action()

    def step(self, action=None) -> typing.Tuple[float, float, bool, bool, dict]:
        """
        Combined function for sending an action and fetching the results.
        :param action: The action to send to the client. If None, the action space will be used.
        :type action: int
        :return: The action result
        :rtype: tuple
        """
        if action is None:
            action = self.get_action()
        self.send_action(action)
        self.fetch_results()
        self.action_space.update(*self.last_update)
        return self.last_update

    def stop(self) -> None:
        """
        Forcibly stops the simulation.
        Sends a 'QUIT' command to the client before closing the connection and socket.
        :return: None
        """
        self.send(QUIT)
        self.conn.close()
        self.socket.close()

def make(host='0.0.0.0', port=7234, algorithm='random', seed=None):
    if seed is not None:
        random.seed(seed)
    server = Server(host, port, algorithm)
    server.start()
    server.fetch_action_space()
    return server