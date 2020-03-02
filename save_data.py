import rosbag
import numpy as np
from performance import parse_bag
import csv
import matplotlib.pyplot as plt

minsub = 34
numsubs = 35
# subID = '01'
list_of_complete_datasets =[1,7,8,9,11,13,14,15,17,18,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37]
list_of_complete_datasets =[1,7,8,9,11,13,14,15,18,20,21,22,24,25,26,27,28,29,30,31,32,33,34,35,36,37]


file = "performance.csv"
columns = ['Subject','Control','Complexity','Lives','Treasure']
with open(file,'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter=',')
    testwriter.writerow(columns)

for sub in range(minsub, numsubs+1):
    found = False
    for i in range(len(list_of_complete_datasets)):
        # print(list_of_complete_datasets[i])
        if sub==list_of_complete_datasets[i]:
            found = True
    if found == True:
        print(sub)
        if sub<10:
            subID = '0' + str(sub)
        else:
            subID = str(sub)

        environments = ['low','high']
        control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
        # control_type = ['none','waypoint','directergodic','sharedergodic','autoergodic']
        # autonomy = ['direct','shared','auto']

        player_lives = np.zeros(10)
        treasures_found = np.zeros(10)
        numeach = np.zeros(10)
        for env in range(0, len(environments)):
            for con in range(0, len(control)):
                try:
                    filename = '/media/murpheylab/Elements/sub' + subID + '/' + subID + '_' + control[con] + '_' + environments[env] + '.bag'
                    # filename = '/home/murpheylab/Desktop/sub' + subID + '/' + subID + '_' + control[con] + '_' + environments[env] + '.bag'
                    # filename = '/home/murpheylab/Desktop/sub' + subID + '/repaired.bag'
                    print(filename)
                    game_data = parse_bag(filename,environments[env])
                    print(game_data.lives)
                    if game_data.game_lives!=game_data.lives:
                        print('game_lives: ',game_data.game_lives,'lives: ',game_data.lives)

                    # Store data into matrix
                    if environments[env] =='low':
                        player_lives[con] = game_data.lives
                        treasures_found[con] = game_data.treasures
                        numeach[con] += 1
                    else:
                        player_lives[con+5] = game_data.lives
                        treasures_found[con+5] = game_data.treasures
                        numeach[con+5] += 1
                    row = [subID,control[con],environments[env],game_data.lives,game_data.treasures]
                    with open (file,'a') as csvfile:
                        testwriter = csv.writer(csvfile,delimiter=',')
                        testwriter.writerow(row)
                except:
                    print('Was unable to open and search bag file for ', environments[env], control[con])
        width = 0.5
        ind = np.arange(10)
        plt.figure(sub)
        p1 = plt.bar(ind,player_lives,width)
        p2 = plt.bar(ind,treasures_found,width,bottom=player_lives)
        plt.ylabel('Score')
        plt.title('Game Performance for Subject ' + subID)
        labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
        plt.xticks(ind, labels)
        plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
        plt.savefig(subID+'_performance.pdf')

        # for i in range(10):
        #     if player_lives[i]>0:
        #         player_lives_all[i,num_combined[i]] = player_lives[i]
        #         treasures_found_all[i,num_combined[i]] = treasures_found[i]
        #         num_combined[i] += numeach[i]


# if (numsubs - minsub)>0:
#     width = 0.5
#     ind = np.arange(10)
#     plt.figure(100)
#     player_lives_mean = np.zeros(10)
#     player_lives_std = np.zeros(10)
#     treasures_found_mean = np.zeros(10)
#     treasures_found_std = np.zeros(10)
#     for i in range(10):
#         player_lives_mean[i] = np.mean(player_lives_all[i,0:num_combined[i]])
#         print(num_combined[i],player_lives_all[i,0:num_combined[i]],player_lives_mean[i])
#         player_lives_std[i] = np.std(player_lives_all[i,0:num_combined[i]])/np.sqrt(num_combined[i])
#         treasures_found_mean[i] = np.mean(treasures_found_all[i,0:num_combined[i]])
#         treasures_found_std[i] = np.std(treasures_found_all[i,0:num_combined[i]])/np.sqrt(num_combined[i])
#
#     p1 = plt.bar(ind,player_lives_mean,width,yerr=player_lives_std)
#     p2 = plt.bar(ind,treasures_found_mean,width,bottom=player_lives_mean,yerr=treasures_found_std)
#     plt.ylabel('Score')
#     plt.title('Aggregate Game Performance')
#     labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
#     plt.xticks(ind, labels)
#     plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
#     plt.savefig('combined_performance.pdf')

plt.show()
