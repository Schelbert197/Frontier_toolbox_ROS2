import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt


def load_yaml_metadata(yaml_file):
    """Function to load the metadata from a YAML file"""
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    return data


def load_map(pgm_file):
    """Function to load the map from a .pgm file"""
    return cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)


def align_maps(img1, img2):
    """Function to crop or resize the images to match dimensions"""
    h1, w1 = img1.shape
    h2, w2 = img2.shape

    if h1 == h2 and w1 == w2:
        return img1, img2
    else:
        # Resize larger map to match the smaller map
        new_h, new_w = min(h1, h2), min(w1, w2)
        resized_img1 = cv2.resize(
            img1, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        resized_img2 = cv2.resize(
            img2, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        return resized_img1, resized_img2


def compare_maps(img1, img2, threshold=50):
    """Function to compare two occupancy grids"""
    # Compute the absolute difference between the maps
    diff = cv2.absdiff(img1, img2)

    # Calculate the percentage of differing cells
    num_differences = np.sum(diff > threshold)
    total_cells = img1.shape[0] * img1.shape[1]
    percentage_diff = (num_differences / total_cells) * 100

    # Highlight differences in red for visualization
    color_diff_map = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    color_diff_map[diff > threshold] = [0, 0, 255]  # Red for differences

    return percentage_diff, color_diff_map


# Paths to the .yaml and .pgm files for both maps
yaml_file1 = '/home/sags/final_project/src/li-slam/turtlebot_control/maps/map360.yaml'
pgm_file1 = '/home/sags/final_project/src/li-slam/turtlebot_control/maps/map360.pgm'
yaml_file2 = '/home/sags/final_project/src/li-slam/turtlebot_control/maps/map330.yaml'
pgm_file2 = '/home/sags/final_project/src/li-slam/turtlebot_control/maps/map330.pgm'

# Load YAML metadata for both maps
metadata1 = load_yaml_metadata(yaml_file1)
metadata2 = load_yaml_metadata(yaml_file2)

# Load the .pgm files as numpy arrays (grayscale images)
map1 = load_map(pgm_file1)
map2 = load_map(pgm_file2)

# Align the maps by resizing/cropping them to the same size
aligned_map1, aligned_map2 = align_maps(map1, map2)

# Compare the aligned maps and compute percentage difference
percentage_diff, diff_visualization = compare_maps(aligned_map1, aligned_map2)

# Show the comparison result
print(f"Percentage of differing cells: {percentage_diff:.2f}%")
# cv2.imshow('Differences', diff_visualization)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Save the result
cv2.imwrite('difference_map.png', diff_visualization)
# Display the saved image using matplotlib
plt.imshow(cv2.cvtColor(diff_visualization, cv2.COLOR_BGR2RGB))
plt.title('Differences in Maps')
plt.show()
