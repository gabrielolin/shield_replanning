import numpy as np
import matplotlib
matplotlib.use("TkAgg", force=True)

import matplotlib.pyplot as plt
def sample_points_in_ellipse(n_points, a=0.05, b=0.2):
    """
    Generates points uniformly distributed inside an ellipse defined by:
      (y/a)^2 + (z/b)^2 <= 1
      
    Parameters:
      n_points: int, number of points to sample
      a: float, semi-axis length along y (default 0.05)
      b: float, semi-axis length along z (default 0.2)
      
    Returns:
      points: numpy array of shape (n_points, 2), where each row is [y, z].
    """
    # Generate n_points random angles between 0 and 2*pi
    angles = np.random.uniform(0, 2*np.pi, n_points)
    # Generate n_points radii with correct weighting for uniform area distribution.
    radii = np.sqrt(np.random.uniform(0, 1, n_points))
    
    # Compute y and z coordinates by scaling the unit circle.
    y = a * radii * np.cos(angles)
    z = b * radii * np.sin(angles)
    
    # Combine y and z into a (n_points x 2) array.
    points = np.column_stack((y, z))
    return points

if __name__ == '__main__':
    # Number of points to generate
    n_points = 1000
    points = sample_points_in_ellipse(n_points)
    
    # Plot the points to visualize the ellipse
    plt.figure()
    plt.scatter(points[:, 0], points[:, 1], s=10, alpha=0.5)
    plt.xlabel('y')
    plt.ylabel('z')
    plt.title('Uniformly Sampled Points in the Ellipse')
    plt.axis('equal')
    plt.show()