import numpy as np
import time
import matplotlib.pyplot as plt
 
def db():
    import pdb
    pdb.set_trace()

def load_mat_from_txt(filename):
    try:
        # Load the matrix from the text file
        matrix = np.loadtxt(filename)
        return matrix
    
    except Exception as e:
        print(f"Error: {e}")
        return None
    
planners = ["ours", "epase", "rrtc"]
for planner in planners:
    fname = "../logs/exp_"+planner+"_full.txt"
    result = load_mat_from_txt(fname)

    # find solution percentage
    inv_ind = [2, 3, 4, 11, 12, 13, 14, 15, 16, 18, 21, 28, 31, 32, 34, 36, 37, 39, 40, 41, 43, 45, 46, 49, 51, 52, 53, 57, 59, 61, 62, 63, 64, 65, 67, 71, 74, 75, 77, 78, 79, 88, 89, 92, 96, 98, 99, 100, 102, 104, 106, 107, 109, 110, 111, 113, 114, 116, 119, 130, 132, 134, 135, 136, 139, 140, 143, 144, 147, 149, 151, 152, 154, 155, 156, 158, 160, 162, 163, 164, 166, 168, 169, 171, 172, 175, 176, 178, 180, 183, 184, 186, 190, 191, 193, 195, 204, 206, 209, 211, 215, 216, 218, 220, 221, 223, 224, 227, 228, 232, 235, 236, 238, 240, 243, 245, 247, 248, 249, 250, 252, 254, 257, 259, 260, 264, 265, 266, 267, 268, 270, 271, 273, 275, 279, 280, 281, 282, 288, 291, 293, 298, 299, 300, 309, 310, 311, 312, 313, 314, 315, 316, 319, 322, 323, 325, 327, 328, 329, 332, 333, 336, 338, 340, 341, 342, 345, 348, 350, 356, 357, 358, 359, 360, 361, 362, 365, 368, 370, 372, 380, 381, 384, 385, 387, 389, 390, 394, 396, 397, 399]
    inv_ind = [num - 1 for num in inv_ind]
    result = np.delete(result, inv_ind, axis=0)
    mask = result[:,0] > 0
    f_result = result[mask]
    num_find = 209 - np.sum(result[:,0] < 0)
    num_succ = np.sum(f_result[:,0] + f_result[:,2] < f_result[:,3])
    num_succ_zero = np.sum(f_result[:,2] < f_result[:,3])
    print("######################################################")
    print("Planner: "+planner)
    print("Avg Projectile TOF: {:f}".format(np.mean(result[:,3])*1000))
    print("Solution rate: {:f}".format(num_find/209))
    print("Success rate: {:f}".format(num_succ/209))
    print("Success rate (zero query): {:f}".format(num_succ_zero/209))
    print("Query time: {:f} +- {:f}".format(np.mean(f_result[:,0])*1000, np.std(f_result[:,0])*1000))
    print("Execution time: {:f} +- {:f}".format(np.mean(f_result[:,2])*1000, np.std(f_result[:,2])*1000))
    print("######################################################")

x = [1,2,3,4,5,6]

mean1 = [1.09878450e+01,2.45286594e+01,1.61639748e+01,3.06989417e-02,3.63289069e-02,3.37752483e-04]
std1 = [1.11486376e+01,1.08534182e+01,3.26561367e+00,1.81748751e-02,1.13852172e-02,2.56955810e-04]
# meidan = [8.14885415e+00 2.12185549e+01 1.57176980e+01 2.43840340e-02
#  3.36725628e-02 2.76675523e-04]

mean2 = [6.13857649e+01,1.31030856e+02,3.95149762e+01,3.39256054e-01,1.10121684e-01,1.23130519e-02]
std2 = [37.73219284,51.90358065,13.85630565, 1.31420352, 0.25314481, 0.08510465]


mean3 = [1.01898754e+02,1.30208005e+02,4.35708150e+01,3.49161249e-01,1.31243134e-01,1.48411586e-02]
std3 = [39.80303768,49.3047496, 23.53190717, 1.2769507,  0.35255094, 0.11831531]

# Plotting
plt.errorbar(x, mean1, yerr=std1, fmt='-o', capsize=5, label='Data with Error Bars')

# Customizing plot
plt.title('Line Plot with Error Bars')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
# plt.yscale('log')
plt.legend()

# Show plot
plt.grid(True)
plt.show() 
# db()
