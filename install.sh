#!/bin/bash
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get install -y openssh-server
sudo apt-get install -y bluetooth libbluetooth-dev
sudo apt-get install -y python-dev

wget https://bootstrap.pypa.io/get-pip.py
#python get-pip.py
#pip install --upgrade pip
#(note: do not use sudo pip, which can cause problems)
#python -m pip install --user numpy scipy matplotlib pybluez
sudo apt-get install -y python-scipy python-bluez

sudo apt-get install --assume-yes git
git clone http://ci-git.int.osram-light.com/positioning_middleware/rssi_positioning.git

#If need to have java as part of the system
#sudo apt-get install -y default-jre
#sudo apt-get install default-jdk