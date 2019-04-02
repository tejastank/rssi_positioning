import blescan
import sys

import bluetooth._bluetooth as bluez

dev_id = 0
try:
	sock = bluez.hci_open_dev(dev_id)
	print "ble thread started"

except:
	print "error accessing bluetooth device..."
    	sys.exit(1)

blescan.hci_le_set_scan_parameters(sock)
blescan.hci_enable_le_scan(sock)

while True:
	returnedList = blescan.parse_events(sock, 5)
	print "*******************************"
	for indx, beacon in enumerate(returnedList):
		print "-------------------------------"
		segments = [x.strip() for x in beacon.split(',')]
		print "Index: " + str(indx) + ""\
		+ ", " + "MAC: " + segments[0]\
		+ ", " + "UUID: " + segments[1]\
		+ ", " + "Major: " + segments[2]\
		+ ", " + "Minor: " + segments[3]
