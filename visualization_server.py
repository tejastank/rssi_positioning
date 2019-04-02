# to run the server: $ bokeh serve --show visulization_server.py
from bokeh.plotting import figure, curdoc
from bokeh.driving import linear
from bokeh.models import Range1d
import random
import socket

from anchors import Anchors


@linear()
def update(step):
    global connection
    dataString = ""
    dataList = []
    x = 0.0
    y = 0.0
    try:
        dataString = connection.recv(128)#maximum amount of data to be received at once.
        dataList = dataString.split(',')
        x = float(dataList[0])
        y = float(dataList[1])
        print(dataString)
    except:
        pass

    ds.data['x'].append(x)
    ds.data['y'].append(y)
    ds.trigger('data', ds.data, ds.data)


try:
    host = '192.168.1.6'
    port = 5000
    serverAddress = (host, port)
    print("Starting data socket server on %s:%s" % serverAddress)
    dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    dataSocket.bind(serverAddress)
    dataSocket.listen(1)#listen to a maximum number of queued connections of 1
    print("Data socket server on %s:%s is listening now." % serverAddress )
    connection, clientAddress = dataSocket.accept()
    print("Connection established with client with IP: %s:%s" % clientAddress)
except:
    pass

commissioningFileName = "commissionning.dat"
anchors = Anchors(commissioningFileName)

p = figure(plot_width=800, plot_height=800)
r_anchors = p.scatter([x*0.6096 for x in anchors.listOfX], [y*0.6096 for y in anchors.listOfY], size=10, color="black", alpha=0.6)
p.xaxis.axis_label = "X(meter)"
p.yaxis.axis_label = "Y(meter)"
r = p.scatter([], [], size=6, color="firebrick", alpha=0.6)
ds = r.data_source

curdoc().add_root(p)
curdoc().title = "Visualization of positioning results"
# Add a periodic callback to be run every 500 milliseconds
curdoc().add_periodic_callback(update, 500)
