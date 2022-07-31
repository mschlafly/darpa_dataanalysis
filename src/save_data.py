#!/usr/bin/env python

# This code will only save uncorrupted data with clear start & end markers. To
# include data from the ROS bags that are faulty, which are listed in
# unparsable_bags.csv file, one needs to run save_data_subscriber script.

# Specify the folder with the rosbags by changing the variable readDataDIR

# it uses python 2.7.17

import csv
from datetime import datetime
import numpy as np
import os
import utils
from utils.parse_rosbag import parse_bag


###############################################################################
# Decide master folder where to save the data
###############################################################################

localDIR = '/home/murpheylab/catkin_ws/src/darpa_dataanalysis/src/'
writeDataDIR = 'raw_data/'
readDataDIR = localDIR+'HST_data_local/rosbags/'
writeDataPlayback = writeDataDIR+'playback/'

###############################################################################
# Functions for csv file manipulation
###############################################################################

def create_csv(filePath, columns):
    with open(filePath, 'w') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow(columns)

def write_csv_rows(filePath, rows):
    with open(filePath, 'a') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for i in range(1,rows.shape[0]): # skip the first row
            writer.writerow(rows[i,:])
###############################################################################
# Set up csv files
###############################################################################

file = "raw_data/raw_data.csv"
columns = ['Subject', 'Control', 'Complexity', 'Lives', 'Treasure','Input']
create_csv(file, columns)

file_game = "raw_data/gametime.csv"
columns = ['Subject', 'Control', 'Complexity', 'Start-Month', 'Start-Day',
           'Start-Hour', 'Start-Min', 'Start-Sec', 'End-Month', 'End-Day',
           'End-Hour', 'End-Min', 'End-Sec']
create_csv(file_game, columns)

file_missingbags = "raw_data/unparsable_bags.csv"
columns = ['Subject', 'Control', 'Complexity', 'Control', 'Complexity']
create_csv(file_missingbags, columns)

###############################################################################
# Determine what subjects to include
###############################################################################

minsub = 1
maxsub = 42

# Specify specific subjects to skip here.
skipped_subjects = []

environments = ['low', 'high']
control = ['none', 'waypoint',
           'directergodic', 'sharedergodic', 'autoergodic']

###############################################################################
# Main script
###############################################################################

for sub in range(minsub, maxsub+1):
    found = False
    for i in range(len(skipped_subjects)):
        # print(list_of_complete_datasets[i])
        if sub == skipped_subjects[i]:
            found = True
    if found is False:
        print(sub)
        if sub < 10:
            subID = '0' + str(sub)
        else:
            subID = str(sub)

        # For playback data, subject folder if not already created
        sub_folder_DIR = writeDataPlayback + 'Sub' + subID + '/'
        if not os.path.exists(sub_folder_DIR):
            os.makedirs(sub_folder_DIR)

        # Loop through trials
        for env in range(0, len(environments)):
            for con in range(0, len(control)):
                try:
                    trialInfo = subID + '_' + control[con] + '_' + environments[env]
                    filename = readDataDIR + 'sub' + subID + '/' + trialInfo + '.bag'
                    print(filename)

                    # Get game data by parsing the bag using parse_bag.py
                    game_data = parse_bag(filename, sub, environments[env])

                    if game_data.game_complete is True:

                        # # saving performance data
                        # row = [subID, control[con], environments[env],
                        #        game_data.lives, game_data.treasures,
                        #        game_data.input_count]
                        # with open(file, 'a') as csvfile:
                        #     writer = csv.writer(csvfile, delimiter=',')
                        #     writer.writerow(row)
                        # print('Saved row to file: ', row)

                        # # saving gametime.csv data
                        # start_time = datetime.fromtimestamp(game_data.start_time)
                        # end_time = datetime.fromtimestamp(game_data.end_time)
                        # row = [subID, control[con], environments[env],
                        #        start_time.month, start_time.day,
                        #        start_time.hour, start_time.minute, start_time.second,
                        #        end_time.month, end_time.day,
                        #        end_time.hour, end_time.minute, end_time.second]
                        # with open(file_game, 'a') as csvfile:
                        #     writer = csv.writer(csvfile, delimiter=',')
                        #     writer.writerow(row)
                        # print('Saved row to file: ', row)

                        ###########################################################
                        # Saving playback data:
                        ###########################################################
                        # Player position over time
                        file = sub_folder_DIR + trialInfo + "_player.csv"
                        columns = ['Time', 'x', 'y', 'theta']
                        create_csv(file, columns)
                        data_length = len(game_data.player_time)
                        rows = np.concatenate((np.array(game_data.player_time).reshape((data_length,1)),
                                                np.array(game_data.player_posX).reshape((data_length,1)),
                                                np.array(game_data.player_posY).reshape((data_length,1)),
                                                np.array(game_data.player_posTheta).reshape((data_length,1))),axis=1)
                        write_csv_rows(file, rows)

                        # Treasure position over time
                        file = sub_folder_DIR + trialInfo + "_treasure.csv"
                        columns = ['Time', 'x', 'y']
                        create_csv(file, columns)
                        data_length = len(game_data.treas_time)
                        rows = np.concatenate((np.array(game_data.treas_time).reshape((data_length,1)),
                                                np.array(game_data.treas_posX).reshape((data_length,1)),
                                                np.array(game_data.treas_posY).reshape((data_length,1))),axis=1)
                        write_csv_rows(file, rows)

                        # Adv 0 position over time
                        file = sub_folder_DIR + trialInfo + "_adv0.csv"
                        columns = ['Time', 'x', 'y', 'theta']
                        create_csv(file, columns)
                        data_length = len(game_data.adv0_time)
                        rows = np.concatenate((np.array(game_data.adv0_time).reshape((data_length,1)),
                                                np.array(game_data.adv0_posX).reshape((data_length,1)),
                                                np.array(game_data.adv0_posY).reshape((data_length,1)),
                                                np.array(game_data.adv0_theta).reshape((data_length,1))),axis=1)
                        write_csv_rows(file, rows)

                        # Adv 1 position over time
                        file = sub_folder_DIR + trialInfo + "_adv1.csv"
                        columns = ['Time', 'x', 'y', 'theta']
                        create_csv(file, columns)
                        data_length = len(game_data.adv1_time)
                        rows = np.concatenate((np.array(game_data.adv1_time).reshape((data_length,1)),
                                                np.array(game_data.adv1_posX).reshape((data_length,1)),
                                                np.array(game_data.adv1_posY).reshape((data_length,1)),
                                                np.array(game_data.adv1_theta).reshape((data_length,1))),axis=1)
                        write_csv_rows(file, rows)

                        # Adv 2 position over time
                        file = sub_folder_DIR + trialInfo + "_adv2.csv"
                        columns = ['Time', 'x', 'y', 'theta']
                        create_csv(file, columns)
                        data_length = len(game_data.adv2_time)
                        rows = np.concatenate((np.array(game_data.adv2_time).reshape((data_length,1)),
                                                np.array(game_data.adv2_posX).reshape((data_length,1)),
                                                np.array(game_data.adv2_posY).reshape((data_length,1)),
                                                np.array(game_data.adv2_theta).reshape((data_length,1))),axis=1)
                        write_csv_rows(file, rows)

                        # Detected objects
                        file = sub_folder_DIR + trialInfo + "_objects.csv"
                        columns = ['Time', 'id', 'x', 'y']
                        create_csv(file, columns)
                        data_length = len(game_data.object_time)
                        rows = np.concatenate((np.array(game_data.object_time).reshape((data_length,1)),
                                                np.array(game_data.object_id).reshape((data_length,1)),
                                                np.array(game_data.object_posX).reshape((data_length,1)),
                                                np.array(game_data.object_posY).reshape((data_length,1))),axis=1)
                        write_csv_rows(file, rows)

                    elif game_data.game_time > 250:
                        row = [subID, control[con], environments[env],
                               game_data.game_time]
                        with open(file_game, 'a') as csvfile:
                            writer = csv.writer(csvfile, delimiter=',')
                            writer.writerow(row)

                except:
                    # print('------------------------------------------------------------')
                    # print('Was unable to open and search bag file for ', environments[env], control[con])
                    # print('------------------------------------------------------------')

                    # Save trial to list of missing bags
                    row = [subID, con, env, control[con], environments[env]]
                    with open(file_missingbags, 'a') as csvfile:
                        writer = csv.writer(csvfile, delimiter=',')
                        writer.writerow(row)
