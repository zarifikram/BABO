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
An arduino agent that is running a server is required.
### Agent
Should be able to send and receive data on some certain host and port via TCP/IP protocol.

### babopy program
The babopy program will be communicating with the agent via TCP/IP protocol. It is upto the user on how to format the data that is sent and received. Babopy handles the rest.

```python
from babopy import make
import random

def demo_function(data):
    print(data)
    return (1, 1, False, False, {"data": "placeholders"})

simulator = make(demo_function, "192.168.0.1", 8080)

for i in range(1000):
    action = random.ranint(0, 10)
    observation, reward, terminated, truncated, info = simulator.step(action)

simulator.close()
```

Here the `demo_function` would be replaced by the user's function that would use the data received from Arduino and process the data.

Full documentation can be found [here](docs/index.html).