class BaseAlgorithm:
    """ 
    Interface for all algorithms. Any algorithm that will be passed to `babopy.Server` must implement this interface.
    """
    def get_action(self) -> int:
        """
        Returns an action from the action space.
        :return: The next optimal action from the action space
        :rtype: int
        """
        raise NotImplementedError
    
    def update(self, observation: float, reward: float, termination: bool, truncation: bool, info: dict) -> None:
        """
        Updates the algorithm with the latest observation, reward, termination, truncation and info.
        :param observation: The latest observation
        :type observation: float
        :param reward: The latest reward
        :type reward: float
        :param termination: Whether the episode has terminated
        :type termination: bool
        :param truncation: Whether the episode has been truncated
        :type truncation: bool
        :param info: Additional information
        :type info: dict
        :return: None
        """
        raise NotImplementedError
