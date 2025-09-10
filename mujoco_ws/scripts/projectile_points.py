import numpy as np
import math 
import matplotlib.pyplot as plt

theta_initial_degrees = 135
velocity_initial = 11
gravity = 9.8
x_initial = 10

theta_initial = theta_initial_degrees*math.pi/180.0
range = pow(velocity_initial, 2)*math.sin(2*theta_initial)/gravity
print("range ", range)
x_disc = np.linspace(0.0, -range, num=20)
print("x_disc 1", x_disc)
# print("x_disc 2", x_disc*x_disc)

multiplication_term = gravity/(2*pow(velocity_initial, 2)*pow(math.cos(theta_initial), 2))
y_disc = math.tan(theta_initial)*x_disc - multiplication_term*x_disc*x_disc
# print("theta_initial ", theta_initial)
# print("math.tan(theta_initial) ", math.tan(theta_initial))

print("y_disc ", y_disc)


x_final = x_initial + x_disc
print("x_final ", x_final)

plt.plot(x_final, y_disc)
plt.show()