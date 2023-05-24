import matplotlib.pyplot as plt
import numpy as np
import matplotlib.colors

x = np.linspace(0, 1, 10)
print
colors = np.random.rand(10, 10)
print(colors)

fig = plt.figure()
fig.add_subplot(1,1,1)
bot_path=[[1,1],[2,2],[3,3],[4,4],[5,5]]
classes= [[1,0,0],[0,0,1],[1,0,1],[1,1,0],[0,1,0]]
plt.scatter([point[0] for point in bot_path],
          [point[1] for point in bot_path],
          c=classes)
plt.pause(0.1)
plt.show(block=False)