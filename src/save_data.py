#!/usr/bin/env python

# This code will only save uncorrupted data with clear start & end markers. To
# include data from the ROS bags that are faulty, which are listed in
# unparsable_bags.csv file, one needs to run perfromenace_subscriber script.

# Specify the folder with the rosbags by changing the variable readDataDIR

import csv
from datetime import datetime
import numpy as np
import os
from parse_rosbag import parse_bag


###############################################################################
# Decide master folder where to save the data
###############################################################################

writeDataDIR = 'raw_data/'
readDataDIR = '/home/kpopovic/darpa_ws/src/VR_exp_ROS/vr_exp_ros/data/sub'

###############################################################################
# Functions for csv file manipulation
###############################################################################


def create_csv(filePath, columns):
    # creates a csv file with given column names
    with open(filePath, 'w') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow(columns)


def write_csv_columns(filePath, rows):
    # write data arays into separate column
    with open(filePath, 'a') as csvfile:
        writer = csv.writer(csvfile)
        for row in rows:
            writer.writerow(row)
    print('Saved row to file: ', row)


def write_csv_rows(filePath, row):
    # write data into each separate rows
    with open(filePath, 'a') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(row)
    print('Saved row to file: ', row)

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

        # Loop through trials
        for env in range(0, len(environments)):
            for con in range(0, len(control)):
                try:
                    trialInfo = subID + '_' + control[con] + '_' + environments[env]
                    filename = readDataDIR + subID + '/' + trialInfo + '.bag'
                    print(filename)

                    # Get game data by parsing the bag using performance.py
                    game_data = parse_bag(filename, sub, environments[env])

                    if game_data.game_complete is True:

                        # saving performance data
                        row = [subID, control[con], environments[env],
                               game_data.lives, game_data.treasures,
                               game_data.input_count]
                        with open(file, 'a') as csvfile:
                            writer = csv.writer(csvfile, delimiter=',')
                            writer.writerow(row)
                        print('Saved row to file: ', row)

                        # saving gametime.csv data
                        start_time = datetime.fromtimestamp(game_data.start_time)
                        end_time = datetime.fromtimestamp(game_data.end_time)
                        row = [subID, control[con], environments[env],
                               start_time.month, start_time.day,
                               start_time.hour, start_time.minute, start_time.second,
                               end_time.month, end_time.day,
                               end_time.hour, end_time.minute, end_time.second]
                        with open(file_game, 'a') as csvfile:
                            writer = csv.writer(csvfile, delimiter=',')
                            writer.writerow(row)
                        print('Saved row to file: ', row)

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
