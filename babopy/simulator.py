import typing
from .communicator import Communicator
import json


class Simulator:
    """
    Class to process reinforcement learning actions.
    """

    def __init__(self, function: typing.Callable, ip: str, port: int) -> None:
        """
        Constructor for the Simulator class.
        :param function: The function that will process the data received from the agent.
        :param ip: The ip address of the arduino agent.
        :param port: The port to connect.
        :return: Returns nothing.
        """
        self.function: typing.Callable = function
        self.ip: str = ip
        self.port: int = port

    def _make(self):
        """
        Creates the simulator class.
        :return: Returns the simulator.
        """
        self.communicator: Communicator = Communicator(self.ip, self.port)
        self.communicator.start()
        return self

    def step(self, action: int) -> typing.Tuple[float, float, bool, bool, typing.Dict]:
        """
        Performs a step in the environment. Step consists of two parts-
        1. An action is sent to the arduino agent.
        2. The arduino agent sends some data back to the simulator. This data is processed by the processor function.
        :param action: The action to perform.
        :return: Returns the observation(float), reward(float), terminated(bool), truncated(bool) and info(dict).
        """
        self.communicator.send(action)
        return self.function(self.communicator.receive())

    def close(self) -> None:
        """
        Closes the simulator and all other dependencies.
        :return: Returns nothing.
        """
        self.communicator.stop()


def make(
    function: typing.Callable, ip: str = "198.168.0.1", port: int = 8080
) -> Simulator:
    """
    Creates a new simulator object.
    :param function: The function that will process the data received from the agent.
    :param ip: The ip address of the arduino agent. Default is 192.168.0.1
    :param port: The port to connect. Default is 8080.
    :return: Returns the simulator object.
    """
    return Environment(function).make(ip, port)
