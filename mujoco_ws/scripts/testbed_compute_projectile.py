#!/usr/bin/env python

"""
IMPORTANT - to be compiled with python 3

Computing the velocity vector of a point on the projectile 

This code publishes the velocity vector along with the point
whose velocity vector has been computed 

"""
#import rospy
import numpy as np 
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import random
#from shield_planner_msgs.srv import TestbedProjectile
#from shield_planner_msgs.msg import Projectile
import std_msgs.msg
import datetime
import math
import time


g_counter = 0
save_figure_logs = False

def getProjectileDiscretizationPts(projectile_start_x,
    projectile_start_y,
    projectile_start_z,
    projectile_vel_x,
    projectile_vel_y,
    projectile_vel_z, 
    projectile_start_3d_x, 
    projectile_start_3d_y, 
    projectile_start_3d_z):

    time_interval = 0.08

    sx = []
    sy = []
    sz = []

    t = 0.0

    while t < 1.5:

        height_check = projectile_start_3d_z + projectile_vel_z*t - (1.0/2)*9.8*t*t
        if (height_check > -0.05):
            sx.append(projectile_start_3d_x + projectile_vel_x*t + (1.0/2)*0.0*t*t)
            sy.append(projectile_start_3d_y + projectile_vel_y*t + (1.0/2)*0.0*t*t)
            sz.append(height_check)

        t += time_interval

    # range_3d = np.sqrt(np.pow((sx[0] - sz[-1]),2)
    # import pdb; pdb.set_trace()
    range_3d = np.linalg.norm(np.array([sx[0].item(), sy[0], sz[0].item()]) - np.array([sx[-1].item(), sy[-1], sz[-1].item()]))

    # print("range in 3d = ", range_3d)



    # g_counter = projectile_start_x.shape[0]

    if save_figure_logs:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(sx, sy, sz, marker='o', color="green", alpha = 0.4)
        ax.scatter3D(projectile_start_x, projectile_start_y, projectile_start_z, marker='^', color="red", alpha = 0.4)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis') 
        # plt.show()
        plt.savefig('projectile_pts_-{date:%Y-%m-%d_%H:%M:%S}.png'.format(date=datetime.datetime.now()))
        plt.clf()

    print("computed parabola ")
    for i in range(len(sx)):
        print(sx[i], sy[i], sz[i])



def computeParabolaEqn(x_coords, z_coords):
    """
    computeParabolaEqn computes the coefficients (a, b, c) for the equation of  
    the parabola z = ax^2 + bx + c

    x_coords: x coordinates of the sampled points 
    z_coords: z coordinates of the sampled points 

    returns - a, b, c
    """ 

    A = np.hstack([x_coords*x_coords, x_coords, np.ones_like(x_coords)]) 
    b = z_coords
    # coeffs = np.linalg.solve(A, b)
    coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
    return coeffs

def computeLineEqn(x_coords, y_coords, plot_line=False):
    A = np.hstack([x_coords, np.ones_like(x_coords)])
    b = y_coords
    m, c = np.linalg.lstsq(A, b, rcond=None)[0]

    # plot the line 
    if plot_line == True:  
        if save_figure_logs:
            plt.plot(x_coords, y_coords, 'o', label='Original data', markersize=10)
            plt.plot(x_coords, m*x_coords + c, 'r', label='Fitted line')
            plt.legend()
            # plt.show()
            plt.savefig('line_-{date:%Y-%m-%d_%H:%M:%S}.png'.format(date=datetime.datetime.now()))
            plt.clf()

    return m, c


def visualizeParabola3d(range_x, range_y, range_z, parabola_coeffs,
                        px, py, pz):
    """
    visualizeParabola3d visualizes the parabola in 3d. 

    range_x: range of x to be sampled from 
    range_y: range of y to be sampled from 
    range_z: range of z to be sampled from (not used currently)
    parabola_coeffs: a, b, c coeffs of the parabola 

    returns - None

    """

    x_coods_parabola = np.linspace(range_x[0], range_x[1])
    y_coods_parabola = np.linspace(range_y[0], range_y[1]) 
    z_coods_parabola = np.dot(np.array([[i*i, i, 1] 
                        for i in x_coods_parabola]), parabola_coeffs)

    if save_figure_logs:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(x_coods_parabola, y_coods_parabola, z_coods_parabola, marker='o', color="green", alpha = 0.4)
        ax.scatter3D(px[0], py[0], pz[0], marker='^', color="red")
        ax.scatter3D(px[1], py[1], pz[1], marker='o', color="green")
        ax.scatter3D(px[2], py[2], pz[2], marker='*', color="blue")
        
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_aspect('equal')
        plt.savefig('3d_parabola_-{date:%Y-%m-%d_%H:%M:%S}.png'.format(date=datetime.datetime.now()))
        plt.clf()
        
    # import pickle
    # pickle.dump(fig, open('FigureObject.fig.pickle', 'wb')) # This is for Python 3 - py2 may need `file` instead of `open`

def getZRotMatrix(angle):
    z_rot_matrix = np.array([[np.cos(angle[0]), np.sin(angle[0]), 0], 
                             [-np.sin(angle[0]), np.cos(angle[0]), 0], 
                             [0, 0, 1]])

    return z_rot_matrix


def getTransformedPts(px, py, pz, z_rot_matrix):


    # import ipdb; ipdb.set_trace()
    # p0_rot = np.dot(z_rot_matrix, np.array([px[0], py[0], pz[0]]))
    # p1_rot = np.dot(z_rot_matrix, np.array([px[1], py[1], pz[1]]))
    # p2_rot = np.dot(z_rot_matrix, np.array([px[2], py[2], pz[2]]))

    # print("p0_rot ", p0_rot)
    # print("p1_rot ", p1_rot)
    # print("p2_rot ", p2_rot)

    # print("p0_rot[0] ", p0_rot[0])
    # print("p1_rot[0] ", p1_rot[0])
    # print("p2_rot[0] ", p2_rot[0])

    # print("p0_rot[1] ", p0_rot[1])
    # print("p1_rot[1] ", p1_rot[1])
    # print("p2_rot[1] ", p2_rot[1])

    # print("p0_rot[2] ", p0_rot[2])
    # print("p1_rot[2] ", p1_rot[2])
    # print("p2_rot[2] ", p2_rot[2])


    # # px_rot = np.array([p0_rot[0][0][0], p1_rot[0][0][0], p2_rot[0][0][0]]).reshape(3,1)
    # # py_rot = np.array([p0_rot[1][0][0], p1_rot[1][0][0], p2_rot[1][0][0]]).reshape(3,1)
    # # pz_rot = np.array([p0_rot[2][0], p1_rot[2][0], p2_rot[2][0]]).reshape(3,1)


    # px_rot = np.array([p0_rot[0][0], p1_rot[0][0], p2_rot[0][0]]).reshape(3,1)
    # py_rot = np.array([p0_rot[1][0], p1_rot[1][0], p2_rot[1][0]]).reshape(3,1)
    # pz_rot = np.array([p0_rot[2][0], p1_rot[2][0], p2_rot[2][0]]).reshape(3,1)


    points_mat = np.dot(z_rot_matrix, np.vstack((px.T, py.T, pz.T))) 
    px_rot = points_mat[0,:].reshape(-1,1)
    py_rot = points_mat[1,:].reshape(-1,1)
    pz_rot = points_mat[2,:].reshape(-1,1)

    # print("px_rot ", px_rot)
    # print("py_rot ", py_rot)
    # print("pz_rot ", pz_rot)

    return px_rot, py_rot, pz_rot

def visualizeParabola2d(range_x, range_y, range_z, px_rot, py_rot, pz_rot, coeffs, tgt_slope):
    
    x_parabola = np.linspace(range_x[0], range_x[1])
    y_parabola = np.linspace(range_y[0], range_y[1])
    z_parabola = np.dot(np.array([[i*i, i, 1] for i in x_parabola]), coeffs)

    # import ipdb; ipdb.set_trace()

    valid_indices = np.where(z_parabola > 0) 
    
    # plot in 2d
    # fig, ax = plt.subplots()
    # ax.plot(x_parabola[valid_indices[0]], z_parabola[valid_indices[0]], 'b^')
    # ax.plot(px_rot, pz_rot, 'r^')

    # plot in 3d 
    if save_figure_logs:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(x_parabola[valid_indices[0]], np.zeros_like(x_parabola[valid_indices[0]]), 
            z_parabola[valid_indices[0]], 'b^')
        ax.scatter3D(px_rot, np.zeros_like(px_rot), pz_rot, 'r^')

        z_tgt_coords = (x_parabola - px_rot[0])*tgt_slope + pz_rot[0]
        # plt.show()
        plt.savefig('2d_parabola_-{date:%Y-%m-%d_%H:%M:%S}.png'.format(date=datetime.datetime.now()))
        plt.clf()

def getPointsOnPlane(coeffs):
    # calculate the discriminant
    d = (coeffs[1]**2) - (4*coeffs[0]*coeffs[2])

    # find two solutions
    sol1 = (-coeffs[1] - np.sqrt(d))/(2*coeffs[0])[0]
    sol2 = (-coeffs[1] + np.sqrt(d))/(2*coeffs[0])[0]

    return sol1, sol2

def computeProjectileRange(coeffs):
    
    # computing range 
    # for the parabola z = ax^2 + bx + c, compute the values of x for which 
    # z = 0

    sol1, sol2 = getPointsOnPlane(coeffs)

    range = np.abs(sol2 - sol1)
    return range

def computeProjectileVel(range, y_vel_direction, projectile_angle, projectile_plane_angle):

    # convert from (-pi, pi) to (0, 2pi)
    projectile_angle = projectile_angle + np.pi 

    # this factor is used to multiply the mag of velocity 
    # with -1 if angle is more than 90 degrees. 
    x_vel_direction = 1

    # under the assumption that the angle subtended
    # by the tgt at the starting point is on the other side
    # of the parabola
    if (projectile_angle > np.pi/2):
        projectile_angle = np.pi - projectile_angle
        x_vel_direction = -1

    # print("sqrt calc ", (range*9.8)/np.sin(2*projectile_angle))
    speed_sq = (range*9.8)/np.sin(2*projectile_angle)
    if (speed_sq > 0):
        projection_speed = np.sqrt(speed_sq)
    else:
        projection_speed = np.sqrt(-speed_sq)
        rospy.logwarn("u negative")



    x_velocity = x_vel_direction*(projection_speed*np.cos(projectile_angle))*np.cos(projectile_plane_angle)
    y_velocity = y_vel_direction*(projection_speed*np.cos(projectile_angle))*np.sin(projectile_plane_angle)
    z_velocity = projection_speed*np.sin(projectile_angle)

    return x_velocity, y_velocity, z_velocity


def computeProjectileVelRInv(range, y_vel_direction, projectile_angle, projectile_plane_angle, z_rot_matrix):

    # convert from (-pi, pi) to (0, 2pi)
    projectile_angle = projectile_angle + np.pi 

    # this factor is used to multiply the mag of velocity 
    # with -1 if angle is more than 90 degrees. 
    x_vel_direction = 1

    # under the assumption that the angle subtended
    # by the tgt at the starting point is on the other side
    # of the parabola
    if (projectile_angle > np.pi/2):
        projectile_angle = np.pi - projectile_angle
        x_vel_direction = -1

    rospy.logwarn("projectile_angle after shifting %f", projectile_angle)

    speed_sq = (range*9.8)/np.sin(2*projectile_angle)
    if (speed_sq > 0):
        projection_speed = np.sqrt(speed_sq)
    else:
        projection_speed = np.sqrt(-speed_sq)
        rospy.logwarn("u negative")
    
    hor_vel = -projection_speed*np.cos(projectile_angle)
    ver_vel = projection_speed*np.sin(projectile_angle)

    # import ipdb; ipdb.set_trace()
    z_rot_inv_matrix =  np.linalg.inv(z_rot_matrix)
    new_x, new_y, new_z = np.dot(z_rot_inv_matrix, np.array([hor_vel[0], 0, ver_vel[0]])) 

    return new_x, new_y, new_z

def circle_line_segment_intersection(circle_center, circle_radius, pt1, pt2, full_line=True, tangent_tol=1e-9):
    """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2 + dy ** 2)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius ** 2 * dr ** 2 - big_d ** 2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
             cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2)
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
        if not full_line:  # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1]
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:  # If line is tangent to circle, return just one point (as both intersections have same location)
            return [intersections[0]]
        else:
            return intersections


def ifSameQuadrant(pt1, pt2):

    pt1_x = pt1[0]
    pt1_y = pt1[1]
    pt2_x = pt2[0]
    pt2_y = pt2[1]

    if (pt1_x > 0 and pt1_y > 0):
        if (pt2_x > 0 and pt2_y > 0):
            # print ("both points lie in First quadrant")
            return True
 
    elif (pt1_x < 0 and pt1_y > 0):
        if (pt2_x < 0 and pt2_y > 0):
            # print ("both points lie in Second quadrant")
            return True
         
    elif (pt1_x < 0 and pt1_y < 0):
        if (pt2_x < 0 and pt2_y < 0):
            # print ("both points lie in Third quadrant")
            return True
     
    elif (pt1_x > 0 and pt1_y < 0):
        if (pt2_x > 0 and pt2_y < 0):
            # print ("both points lie in Fourth quadrant")
            return True

    # print("returning false ")
    return False


# def compute_projectile_cb():
def compute_projectile_cb(req):

    # import ipdb; ipdb.set_trace()

    # print("req.if_testbed.data ", req.if_testbed.data)
    rospy.loginfo("Inside the compute_projectile_cb...")
    # rospy.loginfo("If testbed mode %r ", req.if_testbed.data)

    # start_time = time.process_time()

    if(req.if_testbed.data):
        # we have 2 points and a distance
        # we need to compute 3 points from that information 
        px = np.array([req.points.poses[0].position.x, 
                       req.points.poses[1].position.x]).reshape(2,1)
        py = np.array([req.points.poses[0].position.y, 
                       req.points.poses[1].position.y]).reshape(2,1)
        pz = np.array([req.points.poses[0].position.z, 
                       req.points.poses[1].position.z]).reshape(2,1)

        sampled_distance = req.radius.data

        ret_circ = circle_line_segment_intersection((0,0), sampled_distance, (px[0], py[0]), (px[1], py[1]), full_line=True)

        # distance of inner point from ret circle points 
        # dist0 = np.sqrt(np.pow((ret_circ[0][0] - px[0]), 2) + np.pow((ret_circ[0][1] - py[0]), 2))
        # dist1 = np.sqrt(np.pow((ret_circ[1][0] - px[0]), 2) + np.pow((ret_circ[1][1] - py[0]), 2))


        inner_pt = np.array([px[0], py[0]]).reshape(2,)
        outer_pt = np.array([px[1], py[1]]).reshape(2,)
        circle_pt1 = np.array([ret_circ[0][0], ret_circ[0][1]]).reshape(2,)
        circle_pt2 = np.array([ret_circ[1][0], ret_circ[1][1]]).reshape(2,)
        print("inner_pt ", inner_pt)
        print("outer_pt ", outer_pt)
        print("circle_pt0 = ", circle_pt1)
        print("circle_pt1 = ", circle_pt2)

        dist_inner_c1 = dist = np.sqrt((inner_pt[0] - circle_pt1[0])**2 +
                                    (inner_pt[1] - circle_pt1[1])**2)

        dist_inner_c2 = dist = np.sqrt((inner_pt[0] - circle_pt2[0])**2 +
                                    (inner_pt[1] - circle_pt2[1])**2)

        dist_outer_c1 = dist = np.sqrt((outer_pt[0] - circle_pt1[0])**2 +
                                    (outer_pt[1] - circle_pt1[1])**2)

        dist_outer_c2 = dist = np.sqrt((outer_pt[0] - circle_pt2[0])**2 +
                                    (outer_pt[1] - circle_pt2[1])**2)

        dist_outer_inner = dist = np.sqrt((outer_pt[0] - inner_pt[0])**2 +
                                       (outer_pt[1] - inner_pt[1])**2)


        if (dist_inner_c1 == dist_outer_inner + dist_outer_c1):
            pt_idx = 0
            print("adding circle_pt0")
        else:
            pt_idx = 1
            print("adding circle_pt1")

        px = np.vstack((ret_circ[pt_idx][0], px))    
        py = np.vstack((ret_circ[pt_idx][1], py))    
        pz = np.vstack((0.0, pz))

        # if (ifSameQuadrant(ret_circ[0], (px[0], py[0]))):
        #     px = np.vstack((ret_circ[0][0], px))    
        #     py = np.vstack((ret_circ[0][1], py))    
        #     pz = np.vstack((0.0, pz))
        # elif (ifSameQuadrant(ret_circ[1], (px[1], py[1]))):
        #     px = np.vstack((ret_circ[1][0], px))    
        #     py = np.vstack((ret_circ[1][1], py))    
        #     pz = np.vstack((0.0, pz))
        # else:
        #     print("Error No pt on circle lies in same quad ")

    else:
        # we have 3 points given from vision module 
        # read 3 points on the projectile
        # arranged as x-coods, y-coords and z-coods arrays
        # print("running vision pipeline")
        # print("len(req.points.poses) = ", len(req.points.poses))
        # print(req.points.poses[0].position.x)
        # print(req.points.poses[1].position.x)
        # print(req.points.poses[2].position.x)

        # px = np.array([req.points.poses[0].position.x, 
        #                req.points.poses[1].position.x, 
        #                req.points.poses[2].position.x]).reshape(3,1)
        # py = np.array([req.points.poses[0].position.y, 
        #                req.points.poses[1].position.y, 
        #                req.points.poses[2].position.y]).reshape(3,1)
        # pz = np.array([req.points.poses[0].position.z, 
        #                req.points.poses[1].position.z, 
        #                req.points.poses[2].position.z]).reshape(3,1)

        # px_list = []
        # py_list = []
        # pz_list = []
        # px = np.array(px_list.append(req.points.poses[i].position.x) 
        #                 for i in range(len(req.points.poses))).reshape(len(req.points.poses),1)
        # py = np.array(py_list.append(req.points.poses[i].position.y) 
        #                 for i in range(len(req.points.poses))).reshape(len(req.points.poses),1)
        # pz = np.array(pz_list.append(req.points.poses[i].position.z) 
        #                 for i in range(len(req.points.poses))).reshape(len(req.points.poses),1)

        px_list = []
        for i in range(len(req.points.poses)):
            px_list.append(req.points.poses[i].position.x)
        px = np.array(px_list).reshape(len(req.points.poses), 1)

        py_list = []
        for i in range(len(req.points.poses)):
            py_list.append(req.points.poses[i].position.y)
        py = np.array(py_list).reshape(len(req.points.poses), 1)

        pz_list = []
        for i in range(len(req.points.poses)):
            pz_list.append(req.points.poses[i].position.z)
        pz = np.array(pz_list).reshape(len(req.points.poses), 1)
        

    # print("px = ", px)
    # print("py = ", py)
    # print("pz = ", pz)

    # read 3 points on the projectile
    # arranged as x-coods, y-coords and z-coods arrays
    # px = np.array([5.65381434, 0.73, 0.285]).reshape(3,1)
    # py = np.array([-2.00857747, -0.414205,  -0.270111]).reshape(3,1)
    # pz = np.array([0.0, 1.64515, 1.4144]).reshape(3,1)

    range_x = [0, 10]
    range_y = [0, 10]
    range_z = [0, 10]

    parabola_coeffs = computeParabolaEqn(px, pz)

    # visualizeParabola3d(range_x, range_y, range_z, parabola_coeffs,
    #                     px, py, pz)
    
    # computing line equation formed by plane projection 
    m, c = computeLineEqn(px, py, plot_line=False) 

    # computing tranlation of new axis
    axis_translation_x = -c/m
    
    px_translated = px - axis_translation_x

    projectile_plane_angle = np.arctan2(m, 1)
    print("projectile_plane_angle ", projectile_plane_angle)

    # px[0] is the x coordinate of ret circle point
    # we dont use y_vel_direction anymore  
    if (px[0] > 0):
        y_vel_direction = -1
    else:
        y_vel_direction = 1

    # if (projectile_plane_angle > 0.0):
    #     print("reversing y vel direction ")
    #     y_vel_direction = -1

    z_rot_matrix = getZRotMatrix(projectile_plane_angle)

    px_rot, py_rot, pz_rot = getTransformedPts(px_translated, py, pz, z_rot_matrix)    
    print("px_rot ", px_rot)
    print("py_rot ", py_rot)
    print("pz_rot ", pz_rot)
    px_rot = px_rot + axis_translation_x

    parabola_x_z_plane_coeffs = computeParabolaEqn(px_rot, pz_rot)

    sqrt_tmp = parabola_x_z_plane_coeffs[1]**2 - 4*parabola_x_z_plane_coeffs[0]*parabola_x_z_plane_coeffs[2]

    if (sqrt_tmp > 0):
        solution_1 = (-parabola_x_z_plane_coeffs[1] + 
                        math.sqrt(sqrt_tmp)) / (2 * parabola_x_z_plane_coeffs[0])

        solution_2 = (-parabola_x_z_plane_coeffs[1] - 
                        math.sqrt(sqrt_tmp)) / (2 * parabola_x_z_plane_coeffs[0])

    # if (solution_1 > solution_2):
    #     x_origin = solution_1
    # else:
    #     x_origin = solution_2
    print("solution_1 ", solution_1, ", solution_2 ", solution_2)

    if np.absolute(solution_1 - px_rot[0]) < np.absolute(solution_2 - px_rot[0]):
        x_origin = solution_1
    else:
        x_origin = solution_2


    print("x_origin ", x_origin)

    
    # m = 2*a*x_0 + b
    # projectile_slope = 2*parabola_x_z_plane_coeffs[0]*px_rot[0] + \
    #                     parabola_x_z_plane_coeffs[1]
    projectile_slope = 2*parabola_x_z_plane_coeffs[0]*x_origin + \
                    parabola_x_z_plane_coeffs[1]

    print("projectile_slope ", projectile_slope)

    # theta = atan(m)
    projectile_angle = np.arctan2(projectile_slope, 1)

    print("projectile_angle ", projectile_angle)

    visualizeParabola2d(range_x, range_y, range_z, px_rot, py_rot, pz_rot, 
                        parabola_x_z_plane_coeffs, projectile_slope)

    proj_range = computeProjectileRange(parabola_x_z_plane_coeffs)

    # x_vel, y_vel, z_vel = computeProjectileVel(proj_range, y_vel_direction, projectile_slope, projectile_plane_angle)
    x_vel, y_vel, z_vel = computeProjectileVelRInv(proj_range, y_vel_direction, projectile_angle, projectile_plane_angle, z_rot_matrix)

    print("x_vel ", x_vel)
    print("y_vel ", y_vel)
    print("z_vel ", z_vel)

    if z_vel < 0:
        rospy.loginfo("z_vel is -ve ")
        x_vel = - x_vel
        y_vel = - y_vel
        z_vel = - z_vel

    x_origin = x_origin - axis_translation_x

    projectile_start_3d_x, projectile_start_3d_y, projectile_start_3d_z = np.dot(np.linalg.inv(z_rot_matrix), 
        np.array((x_origin[0], 0, 0)))

    projectile_start_3d_x = projectile_start_3d_x + axis_translation_x

    getProjectileDiscretizationPts(px, 
        py, pz, x_vel, y_vel, z_vel, 
        projectile_start_3d_x, projectile_start_3d_y, projectile_start_3d_z)

    projectile_msg = Projectile()

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'odom_combined'
    projectile_msg.header = header

    projectile_msg.object_id = 0

    # projectile_msg.position.x = px[0]
    # projectile_msg.position.y = py[0]
    # projectile_msg.position.z = pz[0]

    projectile_msg.position.x = projectile_start_3d_x
    projectile_msg.position.y = projectile_start_3d_y
    projectile_msg.position.z = projectile_start_3d_z

    projectile_msg.velocity.x = x_vel
    projectile_msg.velocity.y = y_vel
    projectile_msg.velocity.z = z_vel

    print("projectile_msg.position.x ", projectile_msg.position.x)
    print("projectile_msg.position.y ", projectile_msg.position.y)
    print("projectile_msg.position.z ", projectile_msg.position.z)

    print("projectile_msg.velocity.x ", projectile_msg.velocity.x)
    print("projectile_msg.velocity.y ", projectile_msg.velocity.y)
    print("projectile_msg.velocity.z ", projectile_msg.velocity.z)

    # end_time = time.process_time() - start_time

    rospy.loginfo("Finished compute_projectile_cb, returning now... ")
    # rospy.loginfo("Time in py projectile compuation (msec) = %s", end_time*1000)
        
    return projectile_msg



def compute_projectile_server():
    rospy.init_node('projectile_node', anonymous=True)
    s = rospy.Service('compute_projectile_srv', TestbedProjectile, compute_projectile_cb)
    # print("Ready to compute projectiles")
    rospy.loginfo("Starting parabola computation node ")
    rospy.spin()

    # compute_projectile_cb()

if __name__ == '__main__':
    compute_projectile_server()
