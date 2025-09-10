#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_uncertainty_ellipse(ax, mean, cov, n_std=1.0, facecolor='none', edgecolor='red', **kwargs):
    """
    Plot an ellipse representing the covariance (Gaussian uncertainty) on ax.
    
    Parameters:
    - ax: Matplotlib axis to plot on.
    - mean: 2-element array for the (x, y) mean intercept location.
    - cov: 2x2 covariance matrix.
    - n_std: Number of standard deviations (sigma) to determine the ellipse radii.
    - facecolor: Fill color of the ellipse (default: 'none' for transparent).
    - edgecolor: Edge color of the ellipse.
    - kwargs: Additional keyword arguments passed to the Ellipse patch.
    
    Returns:
    - The matplotlib.patches.Ellipse instance added to ax.
    """
    # Compute eigenvalues and eigenvectors of the covariance matrix
    eigvals, eigvecs = np.linalg.eigh(cov)
    # Sort the eigenvalues and eigenvectors in descending order
    order = eigvals.argsort()[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]
    
    # Compute the ellipse angle (in degrees) from the first eigenvector
    angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    
    # Width and height of the ellipse are 2*n_std*sqrt(eigenvalue)
    width, height = 2 * n_std * np.sqrt(eigvals)
    
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle,
                      facecolor=facecolor, edgecolor=edgecolor, **kwargs)
    ax.add_patch(ellipse)
    return ellipse

def main():
    # Define the mean intercept location (x, y) on the plane
    mean = np.array([-1.1, 1.75])
    
    # Define the covariance matrix (uncertainty) of the intercept location
    cov = np.array([[0.05, 0],
                    [0, 0.5]])
    
    # Create a figure and axis for plotting
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Plot the 1-sigma uncertainty ellipse (red)
    plot_uncertainty_ellipse(ax, mean, cov, n_std=1, facecolor='none',
                             edgecolor='red', linewidth=2, label='1-sigma')
    
    # Plot the 2-sigma ellipse (blue dashed)
    plot_uncertainty_ellipse(ax, mean, cov, n_std=2, facecolor='none',
                             edgecolor='blue', linestyle='--', linewidth=1, label='2-sigma')
    
    # Plot the 3-sigma ellipse (green dotted)
    plot_uncertainty_ellipse(ax, mean, cov, n_std=3, facecolor='none',
                             edgecolor='green', linestyle=':', linewidth=1, label='3-sigma')
    
    # Plot the mean intercept location as a black dot
    ax.plot(mean[0], mean[1], 'ko', label='Prior Estimate')
    
    # Optionally, define and plot the robot's workspace as a rectangle.
    # Example workspace limits: x from -1 to 5 and y from 0 to 4.
    workspace = [-2, 2, 0, 4]  # [xmin, xmax, ymin, ymax]
    ax.set_xlim(workspace[0], workspace[1])
    ax.set_ylim(workspace[2], workspace[3])
    workspace_rect = plt.Rectangle((workspace[0], workspace[2]),
                                   workspace[1]-workspace[0],
                                   workspace[3]-workspace[2],
                                   fill=False, edgecolor='gray', linestyle='--',
                                  )
    ax.add_patch(workspace_rect)
    
    # Labeling and grid
    ax.set_xlabel('Y Position')
    ax.set_ylabel('Z Position')
    ax.set_title('Uncertainty in Projectile Intercept Location at plane X = 0m')
    ax.grid(True)
    ax.legend()
    
    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
