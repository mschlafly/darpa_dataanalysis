#!/usr/bin/env python

import rosbag
import numpy as np
import matplotlib.pyplot as plt

lives_metric = False

minsub = 28
numsubs = 29
# subID = '01'
list_of_complete_datasets =[1,7,8,9,11,13,14,15,17,18,20,21,22,23,24,25,26,27,28,29,30,31,32,33,36,37,38]
list_of_complete_datasets =[1,7,8,9,11,13,14,15,18,20,21,22,24,25,26,27,28,29,30,31,32,33,36,37,38]
print(len(list_of_complete_datasets))

num_combined = np.zeros(10, dtype=int)
player_lives_all = np.zeros((10,len(list_of_complete_datasets)))
treasures_found_all = np.zeros((10,len(list_of_complete_datasets)))

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

        player_lives = np.zeros(10)
        treasures_found = np.zeros(10)
        numeach = np.zeros(10)
        for env in range(len(environments)):
            for con in range(len(control)):
                # try:
                # filename = '/media/murpheylab/Elements/sub' + subID + '/' + subID + '_' + control[con] + '_' + environments[env] + '.bag'
                filename = '/home/murpheylab/Desktop/sub' + subID + '/' + subID + '_' + control[con] + '_' + environments[env] + '.bag'
                print(filename)
                bag = rosbag.Bag(filename)

                topic_list = ['/treasure_info','/adversary_1_position','/adversary_2_position','/adversary_3_position','/person_position','/player_info','/client_count']
                disttolooselife = 1.5
                disttoreposition = 8

                lives = 8
                lives_counted = 8
                adversary_1_x = 0
                adversary_1_y = 0
                adversary_1_x_prev = 0
                adversary_1_y_prev = 0
                adversary_1_x_prev2 = 0
                adversary_1_y_prev2 = 0
                count1 = 0
                adversary_2_x = 0
                adversary_2_y = 0
                adversary_2_x_prev = 0
                adversary_2_y_prev = 0
                adversary_2_x_prev2 = 0
                adversary_2_y_prev2 = 0
                count2 = 0
                adversary_3_x = 0
                adversary_3_y = 0
                adversary_3_x_prev = 0
                adversary_3_y_prev = 0
                adversary_3_x_prev2 = 0
                adversary_3_y_prev2 = 0
                count3 = 0
                player_x = 30
                player_y = 30
                player_x_prev = 30
                player_y_prev = 30
                count_player = 0
                found1 = False
                found2 = False
                found3 = False
                treasures = 0
                game_time = 5*60 + 60 # corresponds to the game being over
                client_connected = False
                for topic, msg, t in bag.read_messages(topics=topic_list):
                    if topic == topic_list[0]:
                        if (msg.treasure_count>treasures):
                            treasures = msg.treasure_count
                    elif topic == topic_list[1]:
                        if count1>10:
                            adversary_1_x_prev2 = adversary_1_x
                            adversary_1_y_prev2 = adversary_1_y
                            adversary_1_x_prev = msg.xpos
                            adversary_1_y_prev = msg.ypos
                            count1 = 0
                        count1 += 1
                        # vec_adv1_facing = [msg.xpos-adversary_1_x,msg.ypos-adversary_1_y]
                        adversary_1_x = msg.xpos
                        adversary_1_y = msg.ypos
                        # vec_adv1_to_player = [player_x-adversary_1_x,player_y-adversary_1_y]
                        # vec_adv1_to_player /= np.norm(vec_adv1_to_player)
                    elif topic == topic_list[2]:
                        if count2>10:
                            adversary_2_x_prev2 = adversary_2_x
                            adversary_2_y_prev2 = adversary_2_y
                            adversary_2_x_prev = msg.xpos
                            adversary_2_y_prev = msg.ypos
                            count2 = 0
                        count2 += 1
                        adversary_2_x = msg.xpos
                        adversary_2_y = msg.ypos
                    elif topic == topic_list[3]:
                        if count3>10:
                            adversary_3_x_prev2 = adversary_3_x
                            adversary_3_y_prev2 = adversary_3_y
                            adversary_3_x_prev = msg.xpos
                            adversary_3_y_prev = msg.ypos
                            count3 = 0
                        count3 += 1
                        adversary_3_x = msg.xpos
                        adversary_3_y = msg.ypos
                    elif topic == topic_list[4]:
                        if count_player>10:
                            player_x_prev = player_x
                            player_y_prev = player_y
                            count_player = 0
                        count_player += 1
                        player_x = msg.xpos
                        player_y = msg.ypos
                    elif topic == topic_list[5]:
                        if (msg.lives_count<lives_counted):
                            lives_counted = msg.lives_count
                            print('Player now has ',lives_counted,' lives at time',(t.secs-game_time)/60.0)
                            print('Snapshot back in time ------')
                            print('player: ',player_x_prev,player_y_prev)
                            #adversary 1
                            dist1 = np.sqrt((adversary_1_x_prev-player_x)**2+(adversary_1_y_prev-player_y)**2)
                            if dist1<=disttolooselife:
                                print('adversary 1 close to player')
                            vec_adv_facing = [adversary_1_x_prev2-adversary_1_x_prev,adversary_1_y_prev2-adversary_1_y_prev]
                            vec_adv_to_player = [player_x_prev-adversary_1_x_prev2,player_y_prev-adversary_1_y_prev2]
                            if np.max(vec_adv_facing)>0:
                                vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                            else:
                                vec_adv_facing = [100,100]
                            vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                            dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                            # print('Player close to adversary 1 at time',(t.secs-game_time)/60.0)
                            if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                                print('adversary 1 facing player')

                            #adversary 2
                            dist2 = np.sqrt((adversary_2_x_prev-player_x)**2+(adversary_2_y_prev-player_y)**2)
                            if dist2<=disttolooselife:
                                print('adversary 2 close to player')
                            vec_adv_facing = [adversary_2_x_prev2-adversary_2_x_prev,adversary_2_y_prev2-adversary_2_y_prev]
                            vec_adv_to_player = [player_x_prev-adversary_2_x_prev2,player_y_prev-adversary_2_y_prev2]
                            if np.max(vec_adv_facing)>0:
                                vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                            else:
                                vec_adv_facing = [100,100]
                            vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                            dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                            # print('Player close to adversary 2 at time',(t.secs-game_time)/60.0)
                            if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                                print('adversary 2 facing player')

                            #adversary 3
                            dist3 = np.sqrt((adversary_3_x_prev-player_x)**2+(adversary_3_y_prev-player_y)**2)
                            if dist3<=disttolooselife:
                                print('adversary 3 close to player')
                                print(adversary_3_x_prev,adversary_3_y_prev)
                            vec_adv_facing = [adversary_3_x_prev2-adversary_3_x_prev,adversary_3_y_prev2-adversary_3_y_prev]
                            vec_adv_to_player = [player_x_prev-adversary_3_x_prev2,player_y_prev-adversary_3_y_prev2]
                            if np.max(vec_adv_facing)>0:
                                vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                            else:
                                vec_adv_facing = [100,100]
                            vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                            dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                            # print('Player close to adversary 3 at time',(t.secs-game_time)/60.0)
                            if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                                print('adversary 3 facing player')

                            # print('adversary_1: ',adversary_1_x_prev,adversary_1_y_prev)
                            # print('adversary_2: ',adversary_2_x_prev,adversary_2_y_prev)
                            # print('adversary_3: ',adversary_3_x_prev,adversary_3_y_prev)
                    elif topic == topic_list[6]:
                        if client_connected==False:
                            if msg.data==1:
                                game_time = t.secs
                                client_connected = True
                        if client_connected==True:
                            if msg.data==0:
                                # If the full trial didn't happen, reset metrics
                                if (t.secs-game_time)/60.0<4.5: # isn't the full trial
                                    lives = 8
                                    lives_counted = 8
                                    adversary_1_x = 0
                                    adversary_1_y = 0
                                    adversary_1_x_prev = 0
                                    adversary_1_y_prev = 0
                                    adversary_1_x_prev2 = 0
                                    adversary_1_y_prev2 = 0
                                    count1 = 0
                                    adversary_2_x = 0
                                    adversary_2_y = 0
                                    adversary_2_x_prev = 0
                                    adversary_2_y_prev = 0
                                    adversary_2_x_prev2 = 0
                                    adversary_2_y_prev2 = 0
                                    count2 = 0
                                    adversary_3_x = 0
                                    adversary_3_y = 0
                                    adversary_3_x_prev = 0
                                    adversary_3_y_prev = 0
                                    adversary_3_x_prev2 = 0
                                    adversary_3_y_prev2 = 0
                                    count3 = 0
                                    player_x = 30
                                    player_y = 30
                                    player_x_prev = 30
                                    player_y_prev = 30
                                    found1 = False
                                    found2 = False
                                    found3 = False
                                    treasures = 0
                                    game_time = 5*60 + 60 # corresponds to the game being over
                                client_connected = False



                    if client_connected==True: # While the game is running
                        dist1 = np.sqrt((adversary_1_x-player_x)**2+(adversary_1_y-player_y)**2)
                        if found1==False:
                            if dist1<=disttolooselife:
                                vec_adv_facing = [adversary_1_x-adversary_1_x_prev,adversary_1_y-adversary_1_y_prev]
                                # print(vec_adv1_facing)
                                # print(np.linalg.norm(vec_adv1_facing))
                                vec_adv_to_player = [player_x-adversary_1_x,player_y-adversary_1_y]
                                if np.max(vec_adv_facing)>0:
                                    vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                                else:
                                    vec_adv_facing = [100,100]
                                vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                                dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                                # print('Player close to adversary 1 at time',(t.secs-game_time)/60.0)
                                if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                                    found1 = True
                                    lives -= 1
                                    print('Life lost: adversary 1 facing player at time',(t.secs-game_time)/60.0)
                        else:
                            if dist1>disttoreposition:
                                found1 = False
                                print('Adversary 1 repositioned')

                        dist2 = np.sqrt((adversary_2_x-player_x)**2+(adversary_2_y-player_y)**2)
                        if found2==False:
                            if dist2<=disttolooselife:
                                vec_adv_facing = [adversary_2_x-adversary_2_x_prev,adversary_2_y-adversary_2_y_prev]
                                vec_adv_to_player = [player_x-adversary_2_x,player_y-adversary_2_y]
                                if np.max(vec_adv_facing)>0:
                                    vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                                else:
                                    vec_adv_facing = [100,100]
                                vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                                dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                                # print('Player close to adversary 2 at time',(t.secs-game_time)/60.0)
                                if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                                    found2 = True
                                    lives -= 1
                                    print('Life lost: adversary 2 facing player at time',(t.secs-game_time)/60.0)

                        else:
                            if dist2>disttoreposition:
                                found2 = False
                                print('Adversary 2 repositioned')


                        dist3 = np.sqrt((adversary_3_x-player_x)**2+(adversary_3_y-player_y)**2)
                        if found3==False:
                            if dist3<=disttolooselife:
                                vec_adv_facing = [adversary_3_x-adversary_3_x_prev,adversary_3_y-adversary_3_y_prev]
                                vec_adv_to_player = [player_x-adversary_3_x,player_y-adversary_3_y]
                                if np.max(vec_adv_facing)>0:
                                    vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                                else:
                                    vec_adv_facing = [100,100]
                                vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                                dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                                print('Player close to adversary 3 at time',(t.secs-game_time)/60.0)
                                if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                                    found3 = True
                                    lives -= 1
                                    print('Life lost: adversary 3 facing player at time',(t.secs-game_time)/60.0)
                        else:
                            if dist3>disttoreposition:
                                found3 = False
                                print('Adversary 3 repositioned')
                    # except:
                    #     print('skip message')

                if lives_counted!=lives:
                    print('lives_counted: ',lives_counted,'lives: ',lives)

                if environments[env] =='low':
                    if lives_metric==True:
                        player_lives[con] = lives_counted
                    else:
                        player_lives[con] = lives
                    treasures_found[con] = treasures
                    numeach[con] += 1
                else:
                    if lives_metric==True:
                        player_lives[con+5] = lives_counted
                    else:
                        player_lives[con+5] = lives
                    player_lives[con+5] = lives
                    treasures_found[con+5] = treasures
                    numeach[con+5] += 1
                # except:
                #     print('Was unable to open and search bag file for ', environments[env], control[con])
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

        for i in range(10):
            if player_lives[i]>0:
                # print(i, num_combined[i])
                player_lives_all[i,num_combined[i]] = player_lives[i]
                treasures_found_all[i,num_combined[i]] = treasures_found[i]
                num_combined[i] += numeach[i]

        # plt.show()

if (numsubs - minsub)>0:
    width = 0.5
    ind = np.arange(10)
    plt.figure(100)
    player_lives_mean = np.zeros(10)
    player_lives_std = np.zeros(10)
    treasures_found_mean = np.zeros(10)
    treasures_found_std = np.zeros(10)
    for i in range(10):
        player_lives_mean[i] = np.mean(player_lives_all[i,0:num_combined[i]])
        print(num_combined[i],player_lives_all[i,0:num_combined[i]],player_lives_mean[i])
        player_lives_std[i] = np.std(player_lives_all[i,0:num_combined[i]])/np.sqrt(num_combined[i])
        treasures_found_mean[i] = np.mean(treasures_found_all[i,0:num_combined[i]])
        treasures_found_std[i] = np.std(treasures_found_all[i,0:num_combined[i]])/np.sqrt(num_combined[i])

    p1 = plt.bar(ind,player_lives_mean,width,yerr=player_lives_std)
    p2 = plt.bar(ind,treasures_found_mean,width,bottom=player_lives_mean,yerr=treasures_found_std)
    plt.ylabel('Score')
    plt.title('Aggregate Game Performance')
    labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
    plt.xticks(ind, labels)
    plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
    plt.savefig('combined_performance.pdf')
