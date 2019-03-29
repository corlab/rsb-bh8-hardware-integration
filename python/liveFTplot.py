import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib import style
import time
import logging
import rsb
import rsb.converter
import rst
import rstsandbox
from rst.dynamics.Wrench_pb2 import Wrench
import numpy as np

# rsb0.16 send '"0.3,0.3,0.3"' 'socket:/debug/ft'

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(111, aspect='equal', projection='3d')
ax1.set_xlim3d(-10, 10)
ax1.set_ylim3d(-10, 10)
ax1.set_zlim3d(-10, 10)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')

xs = [0.0]
ys = [0.0]
zs = [0.0]


def handle(event):
    # print("Received event: %s" % event)
    global xs, ys, zs
    xs = [0.0]
    ys = [0.0]
    zs = [0.0]
    arr = np.array([float(event.data.forces.x), float(event.data.forces.y), float(event.data.forces.z)])

    # x, y, z = event.data.split(',')
    xs.append(arr[0])
    ys.append(arr[1])
    zs.append(arr[2])
    # print(event.metaData.deliverTime - event.metaData.createTime)


def animate(i):
    ax1.clear()
    global xs, ys, zs
    ax1.plot(xs, ys, zs)
    ax1.set_xlim3d(-10, 10)
    ax1.set_ylim3d(-10, 10)
    ax1.set_zlim3d(-10, 10)
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.text(0, 0, 0, "(" + str(xs[len(xs)-1]) +
             "," + str(ys[len(ys)-1]) + ", " + str(zs[len(zs)-1]) + ")", color='red')


if __name__ == '__main__':
    # Pacify logger.
    logging.basicConfig()

    converter = rsb.converter.ProtocolBufferConverter(messageClass=Wrench)
    rsb.converter.registerGlobalConverter(converter)

    # Create a listener on the specified scope. The listener will
    # dispatch all received events asynchronously to all registered
    # handlers.
    with rsb.createListener("/bhand/left/wrenches") as listener:

        # Add a handler to handle received events. Handlers are callable
        # objects with the received event as the single argument.
        listener.addHandler(handle)

        # Wait for events; clean up when interrupted.
        # while True:
        #     time.sleep(1)
        ani = animation.FuncAnimation(fig, animate, interval=180)
        plt.show()
