import rosbag
import numpy as np
from performance import parse_bag
import csv
import matplotlib.pyplot as plt
from datetime import datetime


minsub = 1
maxsub = 42
skipped_subjects = []  # [2,3,4,5,6,10,12,16,19,15,38]
# 15 missing waypoing low and 38 missing directergodic high
# list_of_complete_datasets =[1,7,8,9,11,13,14,15,17,18,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,39,40,41,42]

file = "performance.csv"
columns = ['Subject', 'Control', 'Complexity', 'Lives', 'Treasure']
with open(file, 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(columns)

file_input = "input_data.csv"
columns = ['Subject', 'Control', 'Complexity', 'Input-Count', 'Drone-ID']
with open(file_input, 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(columns)

file_excitement = "excitement_data.csv"
columns = ['Subject', 'Control', 'Complexity', 'Excitement-Time']
with open(file_excitement, 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(columns)

file_game = "gametime.csv"
columns = ['Subject', 'Control', 'Complexity', 'Start-Month', 'Start-Day',
           'Start-Hour', 'Start-Min', 'Start-Sec', 'End-Month', 'End-Day',
           'End-Hour', 'End-Min', 'End-Sec']
with open(file_game, 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(columns)

file_missingbags = "missing_bags_gametime.csv"
columns = ['Subject', 'Control', 'Complexity', 'Control', 'Complexity']
with open(file_missingbags, 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(columns)

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

        environments = ['low', 'high']

        # controls used for getting perfromance data
        control = ['none',
                   'waypoint',
                   'directergodic',
                   'sharedergodic',
                   'autoergodic']

        # udpdated controls relevant for player input only
        # control = ['waypoint', 'directergodic', 'sharedergodic']

        # Loop through trials
        for env in range(0, len(environments)):
            for con in range(0, len(control)):
                try:
                    filename = '/home/kpopovic/Desktop/VR_exp_ROS/vr_exp_ros/data/sub' + subID + '/' + subID + '_' + control[con] + '_' + environments[env] + '.bag'
                    print(filename)

                    # Get game data by parsing the bag using performance.py
                    game_data = parse_bag(filename, sub, environments[env])
                    # print(game_data.game_complete)
                    if game_data.game_complete:

                        # # Prints discrepency in game lives between the number shown to player and counted lives
                        # if game_data.game_lives!=game_data.lives:
                        #     print('Discrepency in lives-- game_lives: ',game_data.game_lives,'lives: ',game_data.lives)
                        # # Prints discrepency in treasures found between the number in ros and counted treasures
                        # if game_data.treasures!=game_data.game_treasures:
                        #     print('Discrepency in treasure-- game_treasures: ',game_data.game_treasures,'treasures: ',game_data.treasures)

                        # # saving performance data
                        # row = [subID, control[con], environments[env],
                        #        game_data.lives, game_data.treasures]
                        # with open(file, 'a') as csvfile:
                        #     writer = csv.writer(csvfile, delimiter=',')
                        #     writer.writerow(row)
                        # print('Saved row to file: ', row)

                        # # saving input data
                        # if control[con] == 'waypoint' or control[con] == 'directergodic' or control[con] == 'sharedergodic':
                        #     row = [subID, control[con], environments[env],
                        #            game_data.input_count, game_data.drone_ID]
                        #     with open(file_input, 'a') as csvfile:
                        #         writer = csv.writer(csvfile, delimiter=',')
                        #         writer.writerow(row)

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


                        # # OPTION 1: saving excitement time data
                        # row = [subID, control[con], environments[env],
                        #        game_data.time_excitement]
                        # with open(file_excitement, 'a') as csvfile:
                        #     writer = csv.writer(csvfile, delimiter=',')
                        #     writer.writerow(row)

                        # OPTION 2: saving excitement file for each Subject
                        # file_individual = '../data/Sub' + subID + '_' + control[con] + '_' + environments[env] + '.csv'
                        # with open(file_individual, 'wb') as csvfile:
                        #     writer = csv.writer(csvfile)
                        #     writer.writerow(game_data.time_excitement)

                    # elif game_data.game_time > 250:
                    #     row = [subID, control[con], environments[env],
                    #           game_data.game_time]
                    #     with open(file_game, 'a') as csvfile:
                    #         writer = csv.writer(csvfile, delimiter=',')
                    #         writer.writerow(row)

                except:
                    # print('------------------------------------------------------------')
                    # print('Was unable to open and search bag file for ', environments[env], control[con])
                    # print('------------------------------------------------------------')

                    # Save trial to list of missing bags
                    row = [subID, con, env, control[con], environments[env]]
                    with open(file_missingbags, 'a') as csvfile:
                        writer = csv.writer(csvfile, delimiter=',')
                        writer.writerow(row)
