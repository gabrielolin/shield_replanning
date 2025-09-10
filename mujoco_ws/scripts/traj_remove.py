import numpy as np
import matplotlib.pyplot as plt
import time
import os

def get_files_in_folder(folder_path):
    try:
        # Initialize an empty list to store file names
        files_with_mtime = []
        
        # Iterate through all the files in the folder
        for root, dirs, files in os.walk(folder_path):
            for file in files:
        # Get the full file path
                file_path = os.path.join(root, file)
                # Get the modification time of the file
                modified_time = os.path.getmtime(file_path)
                # Append the tuple (file_name, modified_time) to the list
                files_with_mtime.append((file_path, modified_time))

        # Sort the list of tuples by modified time
        files_with_mtime.sort(key=lambda x: x[1])

        # Extract the file names from the sorted list
        sorted_files = [file_mtime[0] for file_mtime in files_with_mtime]

        return sorted_files
    
    except Exception as e:
        print(f"Error: {e}")
        return None

def load_traj_from_txt(filename):
    try:
        # Load the matrix from the text file
        matrix = np.loadtxt(filename)
        return matrix
    
    except Exception as e:
        print(f"Error: {e}")
        return None

inv_ind = [2, 3, 4, 11, 12, 13, 14, 15, 16, 18, 21, 28, 31, 32, 34, 36, 37, 39, 40, 41, 43, 45, 46, 49, 51, 52, 53, 57, 59, 61, 62, 63, 64, 65, 67, 71, 74, 75, 77, 78, 79, 88, 89, 92, 96, 98, 99, 100, 102, 104, 106, 107, 109, 110, 111, 113, 114, 116, 119, 130, 132, 134, 135, 136, 139, 140, 143, 144, 147, 149, 151, 152, 154, 155, 156, 158, 160, 162, 163, 164, 166, 168, 169, 171, 172, 175, 176, 178, 180, 183, 184, 186, 190, 191, 193, 195, 204, 206, 209, 211, 215, 216, 218, 220, 221, 223, 224, 227, 228, 232, 235, 236, 238, 240, 243, 245, 247, 248, 249, 250, 252, 254, 257, 259, 260, 264, 265, 266, 267, 268, 270, 271, 273, 275, 279, 280, 281, 282, 288, 291, 293, 298, 299, 300, 309, 310, 311, 312, 313, 314, 315, 316, 319, 322, 323, 325, 327, 328, 329, 332, 333, 336, 338, 340, 341, 342, 345, 348, 350, 356, 357, 358, 359, 360, 361, 362, 365, 368, 370, 372, 380, 381, 384, 385, 387, 389, 390, 394, 396, 397, 399]
folderName = "../logs/trajs_ours_400/"
s_folderName = "../logs/trajs_ours_full/"
files = get_files_in_folder(folderName)
for f in files:
    print(f)
    traj_f = f.split("/")[-1]
    traj_n = traj_f.split("_")[-1]
    if int(traj_n) in inv_ind:
        print("Continuing")
        continue

    save = load_traj_from_txt(f)
    save_f = f.replace("ours_400", "ours_full")
    np.savetxt(save_f, save)