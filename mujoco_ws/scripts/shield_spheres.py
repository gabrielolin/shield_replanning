import numpy as np
import pdb

def shield_spheres():
	size = 0.35
	rad = 0.03
	res = 0.05

	positions =np.arange(-size/2, size/2 + res, res)
	count = 0
	for y in positions:
		for z in positions:
			print(f'        - {{ name: rmsl{count}, x: 0.04, y: {"{:.2f}".format(y)}, z: {"{:.2f}".format(z)}, radius: {"{:.2f}".format(rad)}, priority: 1 }}')
			count += 1
	print(count)

def block_spheres(x = 0, y = 0, z = 0, x_count = 1, y_count = 1, z_count = 1, rad = 0.03, res = 0.05, name = 'l1'):

	x_positions =np.arange(x - (x_count/2 - 0.5)*res, x + (x_count/2)*res , res)
	y_positions =np.arange(y - (y_count/2 - 0.5)*res, y + (y_count/2)*res , res)
	z_positions =np.arange(z - (z_count/2 - 0.5)*res, z + (z_count/2)*res , res)
	# pdb.set_trace()
# + (1 - x_count%2)*res
# + (1 - y_count%2)*res	# pdb.set_trace()
# + (1 - z_count%2)*res
	count = 0
	for x in x_positions:
		for y in y_positions:
			for z in z_positions:
				print(f'        - {{ name: {name}_{count}, x: {"{:.4f}".format(x)}, y: {"{:.4f}".format(y)}, z: {"{:.4f}".format(z)}, radius: {"{:.4f}".format(rad)}, priority: 1 }}')
				count += 1

	# print(count)
	
if __name__ == '__main__':
	shield_spheres()
	# Link-1/ joint 1???
	# block_spheres(0,0,0.065, 4,4,1, 0.065, 0.1, 'l1-1')
	# block_spheres(0,0.215,0.05, 3,1,1, 0.05, 0.08, 'l1-2')
	# block_spheres(0,-0.215,0.05, 3,1,1, 0.05, 0.08, 'l1-3')
	# block_spheres(0.22,0,0.06, 1,3,1, 0.06, 0.09, 'l1-4')
	# block_spheres(0.197,0,0.132, 1,1,1, 0.04, 0.05, 'l1-5')
	# block_spheres(-0.27,0,0.073, 3,6,2, 0.05, 0.07, 'l1-6')
	# block_spheres(-0.25,0.215,0.073, 6,1,4, 0.03, 0.03, 'l1-7')
	# block_spheres(-0.25,-0.215,0.073, 6,1,4, 0.03, 0.03, 'l1-8')
	# block_spheres(0.0,0,0.10, 4,4,1, 0.04, 0.05, 'l1-9')

	# Link-2/joint 2???
	# block_spheres(-0.15,0,-0.3465, 4,4,1, 0.04, 0.05, 'l2-1')
	# block_spheres(-0.15,0,-0.2015, 5,3,2, 0.065, 0.1, 'l2-2')
	# block_spheres(-0.22,0.16,-0.2515, 4,1,1, 0.05, 0.08, 'l2-3')
	# block_spheres(0.10,0.05,-0.2215, 1,4,1, 0.055, 0.08, 'l2-4')
	# block_spheres(-0.05,0,-0.0415, 3,3,2, 0.065, 0.1, 'l2-5')
	# block_spheres(0.13,-0.08,-0.0420, 1,1,3, 0.07, 0.11, 'l2-6')
	
	# Link-3/joint 3???
	# block_spheres(0,-0.05,0.0, 1,3,1, 0.08, 0.13, 'l3-1')
	# block_spheres(0.0,-0.17,-0.25, 1,1,4, 0.08, 0.13, 'l3-2')
	
	# Link-5/joint 5???
	# block_spheres(-0.13, 0.0, 0.0, 3,1,1, 0.07, 0.09, 'l5-1')

	# Link-6/joint 6
	# block_spheres(-0.003, 0.0, 0.0, 1,6,6, 0.005, 0.008, 'l6-1')

