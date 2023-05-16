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

        self.last_update: typing.Tuple[int, float, bool, bool, dict] = (0.0, 0.0, False, False, {})
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
        print(f"Server listening on: {self.host}:{self.port}")
        self.conn, self.addr = self.socket.accept()
        print(f"Client connected: {self.addr}")
    
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
        _data = self.conn.recv(size).decode()
        return _data

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

    def fetch_results(self) -> typing.Tuple[int, float, bool, bool, dict]:
        """
        Fetches the action result from the client. Client sends all the data line by line.
        Each line has some flag followed by values. The last line contains only END flag.
        Flags are, OBS, REW, TERM, TRUN, INFO
        Observation and Reward are mandatory, the others are perceived as default values if
        the client doesn't send anything.

        Example 1:
        OBS 20
        REW 30.2
        TERM 0
        TRUN 0
        INFO success true
        INFO data moved forward
        END
        
        Example 2:
        OBS 8
        REW 40.6
        END
        
        :return: The action result
        :rtype: tuple
        """
        _data = self.receive()
        data = {
            'observation': 0,
            'reward': 0.0,
            'termination': False,
            'truncation': False,
            'info': {}
        }
        _data = _data.split('\n')
        for _d in _data:
            if _d == END_FLAG:
                break
            _type, _value = _d.strip().split(' ', 1)
            if _type == OBSERVATION_FLAG:
                data['observation'] = int(_value)
            elif _type == REWARD_FLAG:
                data['reward'] = float(_value)
            elif _type == TERMINATION_FLAG:
                data['termination'] = False if _value == '0' else True
            elif _type == TRUNCATION_FLAG:
                data['truncation'] = False if _value == '0' else True
            elif _type == INFO_FLAG:
                _key, _val = _value.split(' ', 1)
                data['info'][_key] = _val

        self.last_update = (data['observation'], data['reward'], data['termination'], data['truncation'], data['info'])
        return self.last_update

    def sample(self) -> int:
        """
        Returns the next action based on the action space.
        :return: The next action
        :rtype: int
        """
        return self.action_space.sample()

    def step(self, action=None) -> typing.Tuple[int, float, bool, bool, dict]:
        """
        Combined function for sending an action and fetching the results.
        :param action: The action to send to the client. If None, the action space will be used.
        :type action: int
        :return: The action result
        :rtype: tuple
        """
        if action is None:
            action = self.sample()
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