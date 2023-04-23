# babopy
A python library to interact with Arduino agents and abstract Reinforcement Learning procedures.

## Installation
For now, `babopy` does not have a pip package. You can install it by cloning the repository and moving the `babopy` folder to your python site-packages folder or your python project folder.

```bash
git clone git@github.com:zarifikram/Reinforcement-Learning-Based-Obstacle-Detecting-Crawler-Robot.git
cd Reinforcement-Learning-Based-Obstacle-Detecting-Crawler-Robot
mv babopy /path/to/your/python/site-packages
```

## Usage
The intended use is to have an Arduino agent. However, `babopy` is agnostic of the agent device. It can be used with computer simulations as well.

### Agent
The agent must be able to communicate with `babopy` via TCP/IP protocol. The specific protocol needs to be followed. `babopy.Client` class is available as an implementation of a python agent.

```python
# Demo client
from babopy import Client
client = Client(5)
client.start()
```

### `babopy` server
Before the agent can start communicate with `babopy`, the server needs to be started. It can be found as `babopy.Server` class. However, a `babopy.make` function is given as a wrapper.

```python
# Demo server
from babopy import make
server = make(seed=42)
for i in range(1000):
    action = server.get_action()
    observation, reward, terminated, truncated, info = server.step(action)
server.stop()
```

### Algorithms
`babopy` provides a set of algorithms that can be used to train the agent. The algorithms can be imported from `babopy.algorithms` module. It is possible to pass a custom algorithm to the `babopy.Server` class but it must follow the `babopy.algorithms.BaseAlgorithm` interface.

## More
Examples can be found in the `examples` folder.

Full documentation can be found [here](https://zarifikram.github.io/Reinforcement-Learning-Based-Obstacle-Detecting-Crawler-Robot/babopy/docs/index.html).