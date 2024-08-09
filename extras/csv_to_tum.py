#!/usr/bin/env python3

import pandas as pd

def csv_to_tum(csv_file, tum_file):
    # Read the CSV file
    df = pd.read_csv(csv_file)

    # Open the TUM file for writing
    with open(tum_file, 'w') as f:
        for index, row in df.iterrows():
            # Write each row in the TUM format
            f.write(f"{row['time']} {row['position_x']} {row['position_y']} {row['position_z']} {row['orientation_x']} {row['orientation_y']} {row['orientation_z']} {row['orientation_w']}\n")

if __name__ == '__main__':

    # AMCL CSV file
    csv_file = '/home/rahul/ros/arloc/src/arloc/office_vision_poses.csv' 
    # amcl TUM file
    tum_file = 'office_vision.tum'

    csv_to_tum(csv_file, tum_file)
    print(f"Converted {csv_file} to {tum_file}")

    # gt CSV file
    csv_file = '/home/rahul/ros/arloc/src/arloc/office_ground_vision_truth_poses.csv' 
    # gt TUM file
    tum_file = 'office_gtruth_vision.tum'

    csv_to_tum(csv_file, tum_file)
    print(f"Converted {csv_file} to {tum_file}")
