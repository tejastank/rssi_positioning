class Anchors(object):

	def __init__(self, filename):
		self.listOfFullIDsFromeFile = list()
		self.listOfX = list()
		self.listOfY = list()
		self.listOfZ = list()
		with open(filename, 'r') as f:
			for line in f:
				if line[0] != "#":
					segments = [x.strip() for x in line.split(',')]
					self.listOfFullIDsFromeFile.append(segments[1])
					self.listOfX.append( float(segments[2]) )
					self.listOfY.append( float(segments[3]) )
					self.listOfZ.append( float(segments[4]) )

	def print_list_of_beacon_ids(self):
		print("List of anchors with fullID (UUID-major-minor): ")
		for index, val in enumerate(self.listOfFullIDsFromeFile):
			print("# " + str(index) + ": " + val)

	def print_list_of_beacon_x(self):
		print("List of anchors' x position: ")
		for index, val in enumerate(self.listOfX):
			print("# " + str(index) + ": " + str(val))

	def print_list_of_beacon_y(self):
		print("List of anchors' y position: ")
		for index, val in enumerate(self.listOfY):
			print("# " + str(index) + ": " + str(val))

	def print_list_of_beacon_z(self):
		print("List of anchors' z position: ")
		for index, val in enumerate(self.listOfZ):
			print("# " + str(index) + ": " + str(val))

	def show_debug(self):
		print("Debug info for the commissioned anchors:")
		self.print_list_of_beacon_ids()
		self.print_list_of_beacon_x()
		self.print_list_of_beacon_y()
		self.print_list_of_beacon_z()
