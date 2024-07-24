import pandas as pd

# Load the CSV files
amcl_df = pd.read_csv('husky_amcl.csv')
gt_df = pd.read_csv('jusky_groundtruth.csv')

# Convert the timestamp to a common unit (e.g., seconds) and set it as the index
amcl_df['time'] = pd.to_datetime(amcl_df['time'], unit='s')
gt_df['time'] = pd.to_datetime(gt_df['time'], unit='s')

amcl_df.set_index('time', inplace=True)
gt_df.set_index('time', inplace=True)

# Resample the ground truth to the AMCL rate (e.g., 5 Hz)
gt_df_resampled = gt_df.resample('200ms').nearest()

# Join the two dataframes on the time index
synchronized_df = amcl_df.join(gt_df_resampled, how='inner', lsuffix='_amcl', rsuffix='_gt')

# Save the synchronized dataframe to a new CSV file
synchronized_df.to_csv('synchronized_poses.csv', index_label='time')

print("Synchronized data saved to synchronized_poses.csv")
