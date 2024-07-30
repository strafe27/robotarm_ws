from ikpy.chain import Chain
import numpy as np
import pandas as pd
import math
import time

# Load the robot chain from the URDF file
my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dec.urdf", active_links_mask=[False, True, True, True, True, True, False])

x = 0.03
y = 0.13
z = 0.05

joint_data_df = pd.DataFrame()

# Define the target position (x, y, 0.05) and orientation ([0, -1, 0])
target_position = [x, y, z]
target_orientation = [0, -1, 0]

# Calculate inverse kinematics
ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")


# Convert radians to degrees and round to 2 decimal points
joint_angles_degrees = [round(math.degrees(angle) + 90, 2) for angle in ik.tolist()]

# Create a DataFrame row with formatted data
data_row = pd.DataFrame([joint_angles_degrees + [x, y, z]], columns=[f'Joint {i+1}' for i in range(len(ik))] + ['x', 'y', 'z'])

# Append the row to the joint_data_df
joint_data_df = pd.concat([data_row, joint_data_df], ignore_index=True)

print("Joint Angles DataFrame:")
print(joint_data_df)