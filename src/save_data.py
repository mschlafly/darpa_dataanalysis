import rosbag
import numpy as np
from performance import parse_bag
import csv
import matplotlib.pyplot as plt

minsub = 1
maxsub = 42
skipped_subjects = []#[2,3,4,5,6,10,12,16,19,15,38]
# 15 missing waypoing low and 38 missing directergodic high
# list_of_complete_datasets =[1,7,8,9,11,13,14,15,17,18,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,39,40,41,42]
# print(len(list_of_complete_datasets))

file = "performance.csv"
columns = ['Subject','Control','Complexity','Lives','Treasure']
with open(file,'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter=',')
    testwriter.writerow(columns)

file_missingbags = "missing_bags.csv"
columns = ['Subject','Control','Complexity','Control','Complexity']
with open(file_missingbags,'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter=',')
    testwriter.writerow(columns)


for sub in range(minsub, maxsub+1):
    found = False
    for i in range(len(skipped_subjects)):
        # print(list_of_complete_datasets[i])
        if sub==skipped_subjects[i]:
            found = True
    if found == False:
        print(sub)
        if sub<10:
            subID = '0' + str(sub)
        else:
            subID = str(sub)

        environments = ['low','high']
        control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
        # control_type = ['none','waypoint','directergodic','sharedergodic','autoergodic']
        # autonomy = ['direct','shared','auto']

        # player_lives = np.zeros(10)
        # treasures_found = np.zeros(10)
        # numeach = np.zeros(10)

        # Loop through trials
        for env in range(0, len(environments)):
            for con in range(0, len(control)):
                try:
                    filename = '/home/murpheylab/Desktop/exp_data/sub' + subID + '/' + subID + '_' + control[con] + '_' + environments[env] + '.bag'
                    print(filename)

                    # Get game data by parsing the bag using performance.py
                    game_data = parse_bag(filename,environments[env])
                    print('Lives left: ',game_data.lives)

                    # Prints discrepency in game lives between the number shown to player and counted lives
                    if game_data.game_lives!=game_data.lives:
                        print('Discrepency in lives-- game_lives: ',game_data.game_lives,'lives: ',game_data.lives)

                    # # Store data into matrix
                    # if environments[env] =='low':
                    #     player_lives[con] = game_data.lives
                    #     treasures_found[con] = game_data.treasures
                    #     numeach[con] += 1
                    # else:
                    #     player_lives[con+5] = game_data.lives
                    #     treasures_found[con+5] = game_data.treasures
                    #     numeach[con+5] += 1

                    # Save data to csv
                    # game_data.treasures = game_data.treasures-game_data.end1_treas-game_data.end2_treas
                    # print(game_data.end1_treas,game_data.end2_treas,game_data.treasures)

                    row = [subID,control[con],environments[env],game_data.lives,game_data.treasures]
                    with open(file,'a') as csvfile:
                        testwriter = csv.writer(csvfile,delimiter=',')
                        testwriter.writerow(row)

                    # row = [subID,control[con],environments[env],game_data.end1_lives,game_data.end1_treas]
                    # with open(file,'a') as csvfile:
                    #     testwriter = csv.writer(csvfile,delimiter=',')
                    #     testwriter.writerow(row)
                    # row = [subID,control[con],environments[env],game_data.end2_lives,game_data.end2_treas]
                    # with open(file,'a') as csvfile:
                    #     testwriter = csv.writer(csvfile,delimiter=',')
                    #     testwriter.writerow(row)
                except:
                    print('------------------------------------------------------------')
                    print('Was unable to open and search bag file for ', environments[env], control[con])
                    print('------------------------------------------------------------')

                    # Save trial to list of missing bags
                    row = [subID,con,env,control[con],environments[env]]
                    with open(file_missingbags,'a') as csvfile:
                        testwriter = csv.writer(csvfile,delimiter=',')
                        testwriter.writerow(row)
