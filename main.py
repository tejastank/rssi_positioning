#indoor positioning using BLE beacons and nonlinear least squares optimization
# run the following first to start the visualization server: "bokeh serve --show visualization_server.py"
import sys
import os
import time
import datetime
import socket
import blescan
import bluetooth._bluetooth as bluez
import numpy as np
import numpy.matlib
from anchors import Anchors
from scipy import optimize


def d_i_of_theta(theta, x_i, y_i, z_i ):
	return ( (theta[0] - x_i)**2 + (theta[1] - y_i)**2 + (theta[2] - z_i)**2 ) ** 0.5

# the callable function for a residual term
def f_i_of_theta(theta, x_i, y_i, z_i, r_i ):
    return d_i_of_theta(theta, x_i, y_i, z_i ) - r_i

# the callable function for a residual term squared
def g_i_of_theta(theta, x_i, y_i, z_i, r_i ):
	return f_i_of_theta(theta, x_i, y_i, z_i, r_i ) ** 2

# partial direvatives of g_i_of_theta with respect to theta[0], theta[1], and theta[2].
def gradients(theta, x_i, y_i, z_i, r_i ):
	J = np.empty((r_i.size, theta.size))
	J[:, 0] = 2 * f_i_of_theta( theta, x_i, y_i, z_i, r_i ) * ( theta[0] - x_i ) / d_i_of_theta( theta, x_i, y_i, z_i )
	J[:, 1] = 2 * f_i_of_theta( theta, x_i, y_i, z_i, r_i ) * ( theta[1] - y_i ) / d_i_of_theta( theta, x_i, y_i, z_i )
	J[:, 2] = 2 * f_i_of_theta( theta, x_i, y_i, z_i, r_i ) * ( theta[2] - z_i ) / d_i_of_theta( theta, x_i, y_i, z_i )
	return J


class Scanner(object):

	def __init__(self, uuidList, dev_id=0, numberOfBeaconsToWait=10):
		self.uuidList = uuidList
		self.dev_id = dev_id
		self.__numberOfBeaconsToWait = numberOfBeaconsToWait
		self.sock = None

		try:
			self.sock = bluez.hci_open_dev(self.dev_id)
			print "ble thread started"

		except:
			print "error accessing bluetooth device..."
			sys.exit(1)

		blescan.hci_le_set_scan_parameters(self.sock)
		blescan.hci_enable_le_scan(self.sock)

	def set_number_of_beacons_to_wait(self, numberOfBeaconsToWait):
		self.__numberOfBeaconsToWait = numberOfBeaconsToWait

	def scan(self):
		print("\nScanning...")
		returnedList = blescan.parse_events(self.sock, self.__numberOfBeaconsToWait)
		timestamp = str(time.time())#all scanned beacons would have the same timestamp
		return timestamp, returnedList

class Beacon(object):

	def __init__(self, index, rawString, timestamp):
		self.index = index
		self.rawString = rawString
		self.timestamp = timestamp
		segments = [x.strip() for x in self.rawString.split(',')]
		self.mac = segments[0]
		self.uuid = segments[1]
		self.major = segments[2]
		self.minor = segments[3]
		self.refdbm = float(segments[4])
		self.rssi = float(segments[5])
		self.fullID = self.uuid + self.major + self.minor #strings appending

		self.numberOfOccurence = 0
		self.rssiVariance = None
		self.onFile = False
		self.x = None
		self.y = None
		self.z = None
		self.measuredDistance = None

class DataPoint(object):

	def __init__(self, uuidList, anchors, timestamp, returnedList):
		self.listOfValidUuids = uuidList
		self.anchors = anchors
		self.timestamp = timestamp
		self.returnedList = returnedList
		self.gamma = 2.5# path loss exponent
		self.conversionRatio = 0.6096

		self.listOfBeacons = list()
		self.listOfValidBeacons = list()
		self.listOfValidUniqueBeacons = list() #used for positioning

		self.listOfFullIDsScanned = list()
		self.listOfNumberOfOccurence = list()

		self.numberOfBeacons = 0 # total number of scanned packets (multiple packets from a single beacon is possible)
		self.numberOfValidBeacons = 0 #number of scanned packets from valid beacons (multiple packets from a single beacon is possible)
		self.numberOfValidUniqueBeacons = 0 #number of usable beacons to do positioning

		self.listOfRSSI = list()
		self.listOfRefdbm = list()
		self.listOfPathLoss = list()
		self.listOfMeasuredDistances = list() #distances from the target to the valid and unique beacons.
		self.listOfXPositions = list() #find the values from the commissioning file
		self.listOfYPositions = list() #find the values from the commissioning file
		self.listOfZPositions = list() #find the values from the commissioning file

		self.populate()

	def populate(self):
		for index, rawBeaconString in enumerate(self.returnedList):
			beacon = Beacon( index, rawBeaconString, self.timestamp )
			self.add_beacon(beacon)

		self.filter_beacons_by_uuid()
		self.condense()
		self.sort_beacons()
		self.find_beacon_geo_parameters()
		self.show_debug()

	def add_beacon(self, beacon):
		self.listOfBeacons.append(beacon)
		self.numberOfBeacons += 1

	def filter_beacons_by_uuid(self):
		for beacon in self.listOfBeacons:
			if beacon.uuid in self.listOfValidUuids:
				self.listOfValidBeacons.append(beacon)

		self.numberOfValidBeacons = len(self.listOfValidBeacons)

	def condense(self):
		#if multiple identical beacons exist, combine them to a single one with their rssi value averaged
		#if two beacons with same UUID, Major, and Minor group number, then the two are identical
		for beacon in self.listOfValidBeacons:
			if beacon.fullID not in self.listOfFullIDsScanned:
				self.listOfFullIDsScanned.append(beacon.fullID)
				beacon.numberOfOccurence = 1
				self.listOfValidUniqueBeacons.append(beacon)
				self.listOfNumberOfOccurence.append(1)
				self.numberOfValidUniqueBeacons +=1
			else:
				index = self.listOfFullIDsScanned.index(beacon.fullID)
				self.listOfValidUniqueBeacons[index].rssi = \
				(self.listOfValidUniqueBeacons[index].rssi * self.listOfNumberOfOccurence[index] + \
				                         beacon.rssi * 1)/(self.listOfNumberOfOccurence[index] + 1)
				self.listOfNumberOfOccurence[index] += 1
				self.listOfValidUniqueBeacons[index].numberOfOccurence += 1

	def sort_beacons(self):
		self.listOfValidUniqueBeacons.sort(self.compare_beacons)

	def compare_beacons(self, a, b):
		aValue = int(a.major)*100000 + int(a.minor)
		bValue = int(b.major)*100000 + int(b.minor)
		if  aValue > bValue:
			return 1
		elif aValue == bValue:
			return 0
		else:
			return -1

	def find_beacon_geo_parameters(self):
		for beacon in self.listOfValidUniqueBeacons:
			if beacon.fullID in self.anchors.listOfFullIDsFromeFile:
				beacon.onFile = True
				# beacon.measuredDistance = 10.0 ** ( (beacon.refdbm - beacon.rssi)/(10*self.gamma) )
				beacon.measuredDistance = 10.0 ** ( (-39 - beacon.rssi)/(10*self.gamma) )
				### d = 10^( ( ref - rssi )/( 10*gamma ) )
				whichBeaconOnFile = self.anchors.listOfFullIDsFromeFile.index(beacon.fullID)
				beacon.x = self.conversionRatio * self.anchors.listOfX[whichBeaconOnFile]
				beacon.y = self.conversionRatio * self.anchors.listOfY[whichBeaconOnFile]
				beacon.z = self.conversionRatio * self.anchors.listOfZ[whichBeaconOnFile]
				self.listOfRSSI.append(beacon.rssi)
				self.listOfRefdbm.append(beacon.refdbm)
				self.listOfPathLoss.append(beacon.refdbm - beacon.rssi)
				self.listOfMeasuredDistances.append(beacon.measuredDistance)
				self.listOfXPositions.append(beacon.x)
				self.listOfYPositions.append(beacon.y)
				self.listOfZPositions.append(beacon.z)

	def print_beacons(self):
		for beacon in self.listOfBeacons:
			print(beacon.timestamp, beacon.rawString)

	def print_valid_beacons(self):
		for beacon in self.listOfValidBeacons:
			print(beacon.rawString)

	def print_valid_unique_beacons(self):
		for beacon in self.listOfValidUniqueBeacons:
			print("MAC: %18s, Major: %5d, Minor: %5d, Ref: %6.2f, #occurence: %3d, rssi: %6.2f, d: %5.2f, x: %5.2f, y: %5.2f, z: %5.2f" % (beacon.mac, int(beacon.major), int(beacon.minor), beacon.refdbm, beacon.numberOfOccurence, beacon.rssi, beacon.measuredDistance, beacon.x, beacon.y, beacon.z) )
			#print(beacon.mac, beacon.uuid, beacon.major, beacon.minor, beacon.refdbm, beacon.rssi, beacon.onFile, beacon.x, beacon.y, beacon.z, beacon.measuredDistance)

	# def log_packets(self):

	def show_debug(self):
		print("\n\n----------------------------------------")
		print("\n%d beacon packets in total."                         % self.numberOfBeacons)
		# self.print_beacons()
		print("\n%d valid beacon packets."                            % self.numberOfValidBeacons)
		# self.print_valid_beacons()
		print("\n%d valid and unique beacon packets (rssi averaged)." % self.numberOfValidUniqueBeacons)
		self.print_valid_unique_beacons()

class Solver(object):

	def __init__(self, dataPoint):

		# Definitions of the non-linear minimization problem
		#  -- theta is the target point to be estimated with theta  = (x, y, z)
		#  -- r_i is the measured distances from the target point to the beacon i
		#  -- x_i is the 3D coordinates of the beacon i in the x axis
		#  -- y_i is the 3D coordinates of the beacon i in the y axis
		#  -- z_i is the 3D coordinates of the beacon i in the z axis

		self.x_i = np.asarray(dataPoint.listOfXPositions)
		self.y_i = np.asarray(dataPoint.listOfYPositions)
		self.z_i = np.asarray(dataPoint.listOfZPositions)
		self.r_i = np.asarray(dataPoint.listOfMeasuredDistances)

		# self.theta_initial_guess = np.array([-1.0, 1.8, -1.5])
		# self.theta_initial_guess = np.array([ np.mean(self.x_i), np.mean(self.y_i), -2.7 ])

		seeminglyClosestAnchorIndex = np.argmin(dataPoint.listOfMeasuredDistances)
		print("Closest anchor index and value:")
		print("Anchor: ", seeminglyClosestAnchorIndex)
		print("RSSI: ", dataPoint.listOfRSSI[seeminglyClosestAnchorIndex])
		self.theta_initial_guess = np.array([ self.x_i[seeminglyClosestAnchorIndex], self.y_i[seeminglyClosestAnchorIndex], -2.7 ])

		self.lowerFeasibleBoundTheta = [-10.0, -10.0, -3.0]
		self.upperFeasibleBoundTheta = [ 10.0,  10.0,  3.0]
		'''
		usage: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html#scipy.optimize.least_squares
		(Search for Example of solving a fitting problem, in the follow webpage
		https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html )
		'''
		self.result_dogbox_cauchy  = optimize.least_squares(g_i_of_theta, self.theta_initial_guess, method='dogbox', loss='cauchy',  bounds=(self.lowerFeasibleBoundTheta, self.upperFeasibleBoundTheta), jac=gradients, args=(self.x_i, self.y_i, self.z_i, self.r_i), verbose=1)
		self.result_dogbox_arctan  = optimize.least_squares(g_i_of_theta, self.theta_initial_guess, method='dogbox', loss='arctan',  bounds=(self.lowerFeasibleBoundTheta, self.upperFeasibleBoundTheta), jac=gradients, args=(self.x_i, self.y_i, self.z_i, self.r_i), verbose=1)
		self.result_dogbox_soft_l1 = optimize.least_squares(g_i_of_theta, self.theta_initial_guess, method='dogbox', loss='soft_l1', bounds=(self.lowerFeasibleBoundTheta, self.upperFeasibleBoundTheta), jac=gradients, args=(self.x_i, self.y_i, self.z_i, self.r_i), verbose=1)
		self.result_trf_cauchy     = optimize.least_squares(g_i_of_theta, self.theta_initial_guess, method='trf',    loss='cauchy',  bounds=(self.lowerFeasibleBoundTheta, self.upperFeasibleBoundTheta), jac=gradients, args=(self.x_i, self.y_i, self.z_i, self.r_i), verbose=1)
		self.result_trf_arctan     = optimize.least_squares(g_i_of_theta, self.theta_initial_guess, method='trf',    loss='arctan',  bounds=(self.lowerFeasibleBoundTheta, self.upperFeasibleBoundTheta), jac=gradients, args=(self.x_i, self.y_i, self.z_i, self.r_i), verbose=1)
		self.result_trf_soft_l1    = optimize.least_squares(g_i_of_theta, self.theta_initial_guess, method='trf',    loss='soft_l1', bounds=(self.lowerFeasibleBoundTheta, self.upperFeasibleBoundTheta), jac=gradients, args=(self.x_i, self.y_i, self.z_i, self.r_i), verbose=1)

		# self.result = self.result_trf_soft_l1.x
		trustInitalGuessRatio = 0.25
		self.result = (self.result_trf_soft_l1.x*(1-trustInitalGuessRatio) + self.theta_initial_guess*trustInitalGuessRatio)

		float_formatter = lambda x: "%07.2f" % x
		np.set_printoptions(formatter={'float_kind':float_formatter})
		print("r_i measurements:                  ", self.r_i)
		print("initial guess:                     ", self.theta_initial_guess)
		print("Estimated result_dogbox_cauchy:    ", self.result_dogbox_cauchy.x)
		print("Estimated result_dogbox_arctan:    ", self.result_dogbox_arctan.x)
		print("Estimated result_dogbox_soft_l1:   ", self.result_dogbox_soft_l1.x)
		print("Estimated result_trf_cauchy:       ", self.result_trf_cauchy.x)
		print("Estimated result_trf_arctan:       ", self.result_trf_arctan.x)
		print("Estimated result_trf_soft_l1:      ", self.result_trf_soft_l1.x)

class EDMSolver(object):

	def __init__(self, dataPoint):

		self.x_i = np.asarray(dataPoint.listOfXPositions)
		self.y_i = np.asarray(dataPoint.listOfYPositions)
		self.z_i = np.asarray(dataPoint.listOfZPositions)

		self.r_i = np.asarray(dataPoint.listOfMeasuredDistances)

		self.numOfAnchors = len(self.x_i)
		self.listOfAnchorIndices = np.arange(self.numOfAnchors)

		#add the target as an extra point at the end in the system
		self.x_i = np.append(self.x_i, 0.0)
		self.y_i = np.append(self.y_i, 0.0)
		self.z_i = np.append(self.z_i, 0.0)
		self.r_i = np.append(self.r_i, 0.0)

		#constract the big X matrix
		self.numOfPoints = len(self.x_i)
		self.X = np.mat( np.reshape( np.concatenate( (self.x_i, self.y_i, self.z_i), axis=0 ), (3, self.numOfPoints ) ) )
		self.Yanchors = self.X[:, self.listOfAnchorIndices]
		self.edm = None
		self.Ghat = None
		self.Xhat = None
		self.XhatFinal = None
		self.result = None

	def get_edm(self):
		X = self.X
		G = X.T*X
		oneV = np.matlib.ones( (self.numOfPoints, 1) )
		diagV = np.mat(np.diag(G)).T#need the tranpose to make it a column vector
		self.edm = diagV*oneV.T - 2*G + oneV*diagV.T
		squaredDistancesToAnchors = np.square(self.r_i)
		self.edm[self.numOfPoints-1] = squaredDistancesToAnchors
		self.edm[:,self.numOfPoints-1] = np.reshape(squaredDistancesToAnchors, (self.numOfPoints, 1) )
		# print("self.edm:")
		# print(self.edm)
		# print("self.r_i:")
		# print(self.r_i)

	def estimate_Ghat(self):
		numOfPoints = self.numOfPoints
		identityMat = np.matlib.identity(numOfPoints, dtype=float)
		oneV = np.matlib.ones((numOfPoints,1))
		J = identityMat - (1/float(numOfPoints))*oneV*oneV.T
		self.Ghat = -0.5*J*self.edm*J

	def estimate_Xhat(self):
		dim = 3
		numOfPoints = self.numOfPoints
		w1, v1 = np.linalg.eig(self.Ghat)
		w2 = np.mat(w1)
		v2 = np.mat(v1)
		sortingIndices = np.argsort(w2)
		indices = np.squeeze(np.array(np.flip(sortingIndices,1)))
		w3 = np.flip(np.sort(w2),1)
		w = w3
		v = v2
		v=v[:, indices]
		w = np.sqrt(w[:,0:dim])
		constructMat = np.mat(np.diagflat(w))
		constructMat = np.real(np.concatenate((constructMat, np.matlib.zeros((dim,numOfPoints-dim))), axis=1))
		Xhat = constructMat*v.T
		Xhat = Xhat.real
		self.Xhat = Xhat

	def rectify_w_anchors(self):
		numOfPoints = self.numOfPoints
		Yanchors = self.Yanchors
		Xa = self.Xhat[:,self.listOfAnchorIndices]
		YBar = Yanchors - Yanchors.mean(1)
		XaBar = Xa - Xa.mean(1)
		XYBarProd = XaBar*YBar.T
		U, s, VT = np.linalg.svd(XYBarProd, full_matrices=True) #VT is actually the V transpose
		V=VT.T
		R = V*U.T
		oneV = np.matlib.ones((numOfPoints,1))
		self.XhatFinal = R*(self.Xhat - Xa.mean(1)*oneV.T) + Yanchors.mean(1)*oneV.T


	def run(self):
		print("===============================================================")
		print("EDM apporach solver:")
		self.get_edm()
		self.estimate_Ghat()
		self.estimate_Xhat()
		self.rectify_w_anchors()
		self.result = np.asarray(self.XhatFinal[:,-1]).reshape((1,3))
		print("EDM approach result:")
		print(self.result)
		# print("X:")
		# print(self.X)
		# print("XhatFinal - X:")
		# print(self.XhatFinal - self.X)
		print("===============================================================")


class Reporter(object):

	def __init__(self, hostIPString, portNumber):

		self.hostIPString = hostIPString
		self.portNumber = portNumber
		self.serverAddress = (self.hostIPString, self.portNumber)
		self.dataSocket = socket.socket()
		self.dataSocket.settimeout(3.0)
		self.setup = False
		try:
			self.dataSocket.connect(self.serverAddress)
			self.setup = True
			print("\nConnection to server established.")
		except:
			print("\nError when setting up a reporter.")
			pass

	def __del__(self):
		self.dataSocket.close()
		print("\nData socket closed.")

	def report(self, result):
		if self.setup == True:
			result = result.flatten()
			print(result)
			try:
				# xStr = "%.2f" % result.x[0]
				# yStr = "%.2f" % result.x[1]
				# zStr = "%.2f" % result.x[2]
				xStr = "%.2f" % result[0]
				yStr = "%.2f" % result[1]
				zStr = "%.2f" % result[2]
				dataStr = xStr + ", " + yStr + ", " + zStr
				self.dataSocket.send(dataStr)
				print("-------------Data reported successfully.\n")
			except:
				print("-------------Error when reporting.\n")
				pass
		else:
			print("-------------Data not reported due to unsuccessful setup.\n")

class Logger(object):

	def __init__(self):
		self.directoryName = "/home/pi/rssi_positioning/logs"
		if not os.path.exists(self.directoryName):
			os.mkdir(self.directoryName)
		self.humanTimestamp = time.strftime("%Y-%m-%d-%H-%M-%S")
		self.logFileName = self.directoryName + "/" + self.humanTimestamp + ".log"
		self.logFile = open(self.logFileName, 'w')
		print(self.logFileName)

	def __del__(self):
		print("Closing log file " + self.logFileName)
		self.logFile.close()
		print("Log file " + self.logFileName + " is closed.")

	def log(self, dataString):
		self.logFile.write(dataString+"\n")

	def logDataPoint(self, dataPoint):
		for beacon in dataPoint.listOfValidBeacons:
			self.log(dataPoint.timestamp + "," + beacon.rawString)


def main():

	uuidList = ["e2c56db5dffb48d2b060d0f5a71096e0"]
	dev_id = 0
	#please give a nubmer below 40, since now we are assuming the packets are received roughly at the same time from the same batch of scan.
	#if the number is high, motion is then significant, rssi values are going to be less accurate.
	numberOfBeaconsToWait = 2000
	commissioningFileName = "/home/pi/rssi_positioning/commissionning.dat"

	host = '192.168.1.6'
	port = 5000
	reporter = Reporter(host, port)

	logger = Logger()

	try:
		anchors = Anchors(commissioningFileName)#in anchors.py
		anchors.show_debug()
		scanner = Scanner( uuidList, dev_id, numberOfBeaconsToWait )
		i = 0
		while True:
			timestamp, returnedList = scanner.scan()
			dataPoint = DataPoint(uuidList, anchors, timestamp, returnedList)
			solver = Solver(dataPoint)
			edmSolver = EDMSolver(dataPoint)
			edmSolver.run()
			logger.logDataPoint(dataPoint)
			result = (solver.result + edmSolver.result)/2
			# reporter.report(solver.result)
			# reporter.report(edmSolver.result)
			reporter.report(result)
			i += 1
			print("Running loop number: ", i)

	except KeyboardInterrupt:
	    print("\nWarning: keyboard interrupt detected, quitting...")

	finally:
		#clean up
		print("Program done.")

if __name__ == "__main__":
	main()



###########
#Code below is one example of using the least_squares method provided by scipy library
###########
# def d_i_of_theta( theta, x_i, y_i, z_i ):
# 	return ( (theta[0] - x_i)**2 + (theta[1] - y_i)**2 + (theta[2] - z_i)**2 ) ** 0.5
#
# #the callable function for a residual term
# def f_i_of_theta( theta, x_i, y_i, z_i, r_i ):
#     return d_i_of_theta(theta, x_i, y_i, z_i ) - r_i
#
# #the callable function for a residual term squared
# def g_i_of_theta( theta, x_i, y_i, z_i, r_i ):
# 	return f_i_of_theta(theta, x_i, y_i, z_i, r_i ) ** 2
#
# #the callable function for the gradients with respect to theta[0], theta[1], and theta[2].
# # partial direvatives of g_i_of_theta
# def gradients( theta, x_i, y_i, z_i, r_i ):
# 	J = np.empty((r_i.size, theta.size))
# 	J[:, 0] = 2 * f_i_of_theta( theta, x_i, y_i, z_i, r_i ) * ( theta[0] - x_i ) / d_i_of_theta( theta, x_i, y_i, z_i )
# 	J[:, 1] = 2 * f_i_of_theta( theta, x_i, y_i, z_i, r_i ) * ( theta[1] - y_i ) / d_i_of_theta( theta, x_i, y_i, z_i )
# 	J[:, 2] = 2 * f_i_of_theta( theta, x_i, y_i, z_i, r_i ) * ( theta[2] - z_i ) / d_i_of_theta( theta, x_i, y_i, z_i )
# 	return J
#
#
#
# #get print format back to defualt please use the next line
# #np.set_printoptions(edgeitems=3,infstr='inf', linewidth=75, nanstr='nan', precision=8, suppress=False, threshold=1000, formatter=None)
# x_i = np.array([  -5.0,   5.0,   -5.0,    5.0])
# y_i = np.array([  -5.0,  -5.0,    5.0,    5.0])
# z_i = np.array([   3.0,   3.0,    3.0,    3.0])
#
# r_i = np.array([ 3 + np.random.normal(0, 0.5),\
# 				 np.sqrt(109) + np.random.normal(0, 1),\
# 				 np.sqrt(109) + np.random.normal(0, 1),\
# 				 np.sqrt(209) + np.random.normal(0, 1)])
#
# theta_initial_guess = np.array([0.0, 0.0, 0.0])
# lowerFeasibleBoundTheta = [-10, -10, 0]
# upperFeasibleBoundTheta = [10, 10, 2.0]
# '''
# usage: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html#scipy.optimize.least_squares
# (Search for Example of solving a fitting problem, in the follow webpage
# https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html )
# '''
# result = optimize.least_squares(g_i_of_theta, theta_initial_guess, bounds=(lowerFeasibleBoundTheta, upperFeasibleBoundTheta), jac=gradients, args=(x_i, y_i, z_i, r_i), verbose=1)
#
# float_formatter = lambda x: "%07.2f" % x
# np.set_printoptions(formatter={'float_kind':float_formatter})
# print("r_i measurements: ", r_i)
# print("initial guess:    ", theta_initial_guess)
# print("Estimated:        ", result.x)
