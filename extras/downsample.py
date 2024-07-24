import pandas as pd

# Load the CSV files
amcl_df = pd.read_csv('/home/rahul/RI_Shen/catkin_ws/src/description/husky_amcl.csv')
gt_df = pd.read_csv('/home/rahul/RI_Shen/catkin_ws/src/description/husky_groundtruth_final.csv')

# Select every 37th row from the ground truth data
gt_df_downsampled = gt_df.iloc[::37, :].copy()

# Reset index to align based on row numbers
amcl_df.reset_index(drop=True, inplace=True)
gt_df_downsampled.reset_index(drop=True, inplace=True)

gt_df_downsampled.loc[:, 'time'] = amcl_df['time']
gt_df_downsampled['time'] = gt_df_downsampled['time'] / 10e12
amcl_df['time'] = amcl_df['time'] / 10e12
# Ensure both DataFrames have the same length
# if len(amcl_df) != len(gt_df_downsampled):
#     print("Error: The lengths of the DataFrames do not match.")
#     exit(1)

# # Concatenate the DataFrames for comparison
# synchronized_df = pd.concat([amcl_df, gt_df_downsampled], axis=1)

# # Save the synchronized dataframe to a new CSV file
# synchronized_df.to_csv('synchronized_poses.csv', index=False)

# Save AMCL poses to .tum format
with open('amcl.tum', 'w') as amcl_file:
    for _, row in amcl_df.iterrows():
        amcl_file.write(f"{row['time']} {row['field.pose.pose.position.x']} {row['field.pose.pose.position.y']} {row['field.pose.pose.position.z']} {row['field.pose.pose.orientation.x']} {row['field.pose.pose.orientation.y']} {row['field.pose.pose.orientation.z']} {row['field.pose.pose.orientation.w']}\n")

# Save ground truth poses to .tum format
with open('ground_truth.tum', 'w') as gt_file:
    for _, row in gt_df_downsampled.iterrows():
        gt_file.write(f"{row['time']} {row['position_x']} {row['position_y']} {row['position_z']} {row['orientation_x']} {row['orientation_y']} {row['orientation_z']} {row['orientation_w']}\n")

print("Synchronized data saved to synchronized_poses.csv")
print("AMCL poses saved to amcl.tum")
print("Ground truth poses saved to ground_truth.tum")
