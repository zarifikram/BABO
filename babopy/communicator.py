import typing
import threading
from queue import Queue
import socket

QUIT = "quit"


class Communicator:
    """
    Class to handle communication between the simulator and the Arduino agent.
    The class uses a python Queue object to receive actions from the simulator.
    """

    def __init__(self, host: str, port: int) -> None:
        """
        Constructor for the Communicator class.
        :param host: The host ip of the arduino agent.
        :param port: The port to connect.
        :return: Returns nothing.
        """
        self.host: str = host
        self.port: int = port
        self.queue: Queue = Queue()
        self.client: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.host, self.port))

    def start(self) -> None:
        """
        Starts a new thread that connects to the arduino agent and waits to receive actions.
        When an action is present in the queue, it is sent to the arduino agent.
        :return: Returns nothing.
        """
        self.thread: threading.Thread = threading.Thread(target=self.run)
        self.thread.start()

    def send(self, message: str) -> None:
        """
        Sends a message to the arduino agent.
        :param message: The message to send.
        :return: Returns nothing.
        """
        self.queue.put(message)

    def receive(self) -> str:
        """
        Receives a message from the arduino agent. Max 1024 bytes.
        :return: Returns the decoded message.
        """
        return self.client.recv(1024).decode()

    def stop(self) -> None:
        """
        Stops the thread and closes the connection.
        :return: Returns nothing.
        """
        self.queue.put(QUIT)
        self.thread.join()
        self.client.close()

    def run(self) -> None:
        """
        The thread function that connects to the arduino agent and waits to receive actions.
        If the action is QUIT the thread is stopped. Otherwise the action is sent to the arduino agent.
        :return: Returns nothing.
        """
        while True:
            message = self.queue.get()
            self.client.send(f"{message}".encode())
            if message == QUIT:
                break
