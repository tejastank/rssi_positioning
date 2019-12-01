## Description
This is the software for developing the indoor positioning using RF RSSI (Bluetooth LE beacons).


## Installation
1. Run install.sh to install the project dependencies. (not completed yet)

## Usage

1. visualization_server.py is supposed to be running on a server with known IP:Port address. This file is only in charge of displaying the results.
2. main.py is supposed to be running on a remote target whose position is to be estimated. The position of the target is estimated by the target and then sent to the server with the known IP:Port address.
3. Please make sure the IP:Port addresses are the same and meaningful for test setup.
4. In order to run the real-time interactive visualization on the server, bokeh package is required. To install please install Anaconda first (https://www.anaconda.com/download/#linux), then install bokeh by
```
$ conda install bokeh
```
5. To run the visualization_server.py use command:
```
$ bokeh serve --show visualization_server.py
```
6. To run the main.py on a raspberry pi, use command:
```
sudo python main.py
```

7. Proper shutdown:
  - First, shutdown the main.py running on the target first by using "Ctrl+c"
  - Then, shutdown the bokeh server by using "Ctrl+c"
  - If sequence is wrong, need to restart the server machine since the port used by the bokeh server will remain occupied.


## Usefull links and origional work

http://inside.mines.edu/fs_home/whereman/papers/Hereman-Murphy-Trilateration-Manual-MCS-11-1991.pdf

https://inside.mines.edu/~whereman/papers/Murphy-MS-Thesis-Complete-1992.pdf


[Trilateration using Java] (https://github.com/lemmingapex/trilateration).

[Levenberg-Marquardt algorithm](http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm) from [Apache Commons Math](http://commons.apache.org/proper/commons-math/).

[iBeacon-Scanner-](https://github.com/switchdoclabs/iBeacon-Scanner-).

[Capture beacons using ubertooth with Wireshark](https://github.com/greatscottgadgets/ubertooth/wiki/Capturing-BLE-in-Wireshark).
