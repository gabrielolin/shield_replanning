import math

# params
L = 0.4			# shield size or original cell size
dp = 0.1		# position tolerance in x,y,z
dR = 0.174533	# roll tolerance	
dP = 0.174533	# yaw/pitch tolerance

dY = dP     # assuming both are equal
red_Y = L * (1 - math.cos(dY))			# reduction due to yaw and pitch
red_R = L * (1 - 1/(math.sin(dR) + math.cos(dR)))	# reduction due to roll

L_new = L - red_Y - red_R - dp

print(L_new)