import typing
import random
from .algorithms import RandomAlgorithm

algo_map = {
    'random': RandomAlgorithm
}

def get_random_state(action: int) -> typing.Tuple[float, float, bool, bool, dict]:
    """
    Returns a random state.
    :return: A random state
    :rtype: typing.Tuple[float, float, bool, bool, dict]
    """
    return random.random(), random.random(), random.choice([True, False]), random.choice([True, False]), {"event": "A random state is sent."}