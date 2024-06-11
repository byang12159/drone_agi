#!/usr/bin/env python3
import re
import numpy as np

def extract_target_pos(log_file_path):
    target_pos_list = []
    vel_cmd_list = []
    curr_state_list = []
    temp_vel_list = []
    temp_state_list = []
    pattern = r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3} - rosout - INFO - Starting velPID ctrl, current state: \[.*\], target pos: \[(.*?)\]"
    pattern_cycle_end = r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3} - rosout - INFO - Exiting velPID loop"
    pattern_mid = r'Publishing VelY to Ctrl:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*Current state\s*:\s*\[([^\]]+)\]'

    cycle_open=True

    with open(log_file_path, 'r') as file:
        for line in file:
            match = re.search(pattern, line)
            match_end = re.search(pattern_cycle_end, line)
            match_mid = re.search(pattern_mid, line)
            if match:
                target_pos = match.group(1)
                target_pos_list.append([float(coord.strip()) for coord in target_pos.split()])
            elif match_end:
                cycle_open=False
            elif match_mid:
                vel_cmd = match_mid.group(1)
                curr_state = match_mid.group(2)
                if cycle_open:
                    temp_vel_list.append(float(vel_cmd))
                    temp_state_list.append([float(coord.strip()) for coord in curr_state.split()])
                else:
                    vel_cmd_list.append(temp_vel_list)
                    curr_state_list.append(temp_state_list)
                    temp_vel_list = []
                    temp_state_list = []
                    temp_vel_list.append(float(vel_cmd))
                    temp_state_list.append([float(coord.strip()) for coord in curr_state.split()])
                    cycle_open=True
            else:
                print("WARNING AT LINE {}".format(line))
    if (len(temp_vel_list)>0):
        vel_cmd_list.append(temp_vel_list)
        curr_state_list.append(temp_state_list)

    return target_pos_list, vel_cmd_list, curr_state_list

# Example usage:
log_file_path = "logfile_deploy2.log"
target_pos_list,vel_cmd_list, curr_state_list = extract_target_pos(log_file_path)
print(len(target_pos_list))
numbers = np.linspace(0,len(target_pos_list),len(target_pos_list))
target_pos_list= np.array(target_pos_list)

print(numbers)
import matplotlib.pyplot as plt
import numpy as np


# Create three subplots vertically arranged
plt.figure(figsize=(8, 6))

# First subplot
plt.subplot(3, 1, 1)  # (rows, columns, panel number)
plt.plot(numbers, target_pos_list[:,0], 'r-')
plt.title('Plot 1')

# Second subplot
plt.subplot(3, 1, 2)
plt.plot(numbers, target_pos_list[:,1], 'g-')
plt.title('Plot 2')

# Third subplot
plt.subplot(3, 1, 3)
plt.plot(numbers, target_pos_list[:,2], 'b-')
plt.title('Plot 3')

# Adjust layout
plt.tight_layout()
plt.show()


print(len(vel_cmd_list))
print(len(curr_state_list))


for i in range(len(vel_cmd_list)):
    curr_state = np.array(curr_state_list[i])
    iterations = np.linspace(0,len(vel_cmd_list[i]),len(vel_cmd_list[i]))
    
    plt.figure(figsize=(10, 6))
    # First subplot
    plt.subplot(4, 1, 1)  # (rows, columns, panel number)
    plt.plot(iterations, vel_cmd_list[i], 'r-')
    plt.title('Plot 1')

    # Second subplot
    plt.subplot(4, 1, 2)
    plt.plot(iterations, curr_state[:,0], 'g-')
    plt.title('Plot 2')

    # Third subplot
    plt.subplot(4, 1, 3)
    plt.plot(iterations, curr_state[:,1], 'b-')
    plt.title('Plot 3')

    plt.subplot(4, 1, 4)
    plt.plot(iterations, curr_state[:,0], 'p-')
    plt.title('Plot 3')

    # Adjust layout
    plt.tight_layout()
    plt.suptitle(target_pos_list[i], fontsize=16)
    plt.show()
    
    
print(vel_cmd_list[0])