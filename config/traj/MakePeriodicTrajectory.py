import math

def fmt(value):
	return "%.3f" % value

period = [4, 2, 4]
radius = 1.5
timestep = 0.02
maxtime = max(period)*3
#timemult = [1, 1, 1]
# timemult = 1
timemult = 1 / 1.5
phase=[0,0,0]
amp = [1,0.4,.5]
center = [0, 0, -2]

def calcPoint(t, coord):
	return math.sin(t * 2 * math.pi / period[coord] + phase[coord]) * radius * amp[coord] + center[coord]
with open('FigureEight.txt', 'w') as the_file:
	t=0
	px, py, pz = calcPoint(0, 0), calcPoint(0, 1), calcPoint(0, 2)
	while t <= maxtime:
		x, y, z = calcPoint(t, 0), calcPoint(t, 1), calcPoint(t, 2)
		the_file.write(fmt(t*timemult) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z))
		# vx = 0
		# vy = 0
		# vz = 0
		######## BEGIN STUDENT CODE

		vx, vy, vz = (x-px)/timestep, (y-py)/timestep, (z-pz)/timestep
		px, py, pz = x, y, z

		######## END STUDENT CODE
		the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz))
		######## EXAMPLE SOLUTION
		# the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt((z-pz)/timestep))
		# px = x
		# py = y
		# pz = z
		######## END EXAMPLE SOLUTION
		
		the_file.write("\n")

		t += timestep

