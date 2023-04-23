from babopy import make
from pprint import pprint

server = make(seed=42)

for i in range(100):
    action = server.get_action()
    observation, reward, terminated, truncated, info = server.step(action)
    pprint({
        'action': action,
        'observation': observation,
        'reward': reward,
        'terminated': terminated,
        'truncated': truncated,
        'info': info
    })

# stop the server
server.stop()