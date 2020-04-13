#!/usr/bin/env python

# import rosbag
import rospy
import numpy as np
import csv
from populate_buildings import populate_building_array
from user_input.msg import treasure_info, person_position, player_info
from std_msgs.msg import Int32

class parse_bag:

    def __init__(self,env):
        rospy.init_node('data_analysis', anonymous=True)
        self.rate = rospy.Rate(1)

        self.disttolooselife = 1.5
        self.disttoreposition = 8
        self.building_array = populate_building_array(env)
        self.reset_game()

        rospy.Subscriber('/treasure_info', treasure_info, self.update_treasure)
        rospy.Subscriber('/adversary_1_position', person_position, self.update_adv_1)
        rospy.Subscriber('/adversary_2_position', person_position, self.update_adv_2)
        rospy.Subscriber('/adversary_3_position', person_position, self.update_adv_3)
        rospy.Subscriber('/person_position', person_position, self.update_person)
        rospy.Subscriber('/player_info', player_info, self.update_player_info)
        rospy.Subscriber('/client_count', Int32, self.update_client)

    def reset_game(self):
        self.lives = 8
        self.game_lives = 8
        self.adversary_1_x = 0
        self.adversary_1_y = 0
        self.adversary_1_x_prev = 0
        self.adversary_1_y_prev = 0
        self.adversary_2_x = 0
        self.adversary_2_y = 0
        self.adversary_2_x_prev = 0
        self.adversary_2_y_prev = 0
        self.adversary_3_x = 0
        self.adversary_3_y = 0
        self.adversary_3_x_prev = 0
        self.adversary_3_y_prev = 0
        self.player_x = 30
        self.player_y = 30
        self.found1 = False
        self.found2 = False
        self.found3 = False
        self.treasures = 0
        self.start_time = 0#rospy.get_time()
        self.client_connected = False

    def update_treasure(self, msg):
        if (msg.treasure_count>self.treasures):
            self.treasures = msg.treasure_count
            print('new treasure')

    def update_adv_1(self, msg):
        self.adversary_1_x_prev = self.adversary_1_x
        self.adversary_1_y_prev = self.adversary_1_y
        self.adversary_1_x = msg.xpos
        self.adversary_1_y = msg.ypos
        if self.client_connected==True:
            t=rospy.get_time()
            self.found1 = self.is_player_found(self.found1, self.adversary_1_x, self.adversary_1_y, self.adversary_1_x_prev, self.adversary_1_y_prev, t)

    def update_adv_2(self, msg):
        self.adversary_2_x_prev = msg.xpos
        self.adversary_2_y_prev = msg.ypos
        self.adversary_2_x = msg.xpos
        self.adversary_2_y = msg.ypos
        if self.client_connected==True:
            t=rospy.get_time()
            self.found2 = self.is_player_found(self.found2, self.adversary_2_x, self.adversary_2_y, self.adversary_2_x_prev, self.adversary_2_y_prev, t)

    def update_adv_3(self, msg):
        self.adversary_3_x_prev = self.adversary_3_x
        self.adversary_3_y_prev = self.adversary_3_y
        self.adversary_3_x = msg.xpos
        self.adversary_3_y = msg.ypos
        if self.client_connected==True:
            t=rospy.get_time()
            self.found3 = self.is_player_found(self.found3, self.adversary_3_x, self.adversary_3_y, self.adversary_3_x_prev, self.adversary_3_y_prev, t)

    def update_client(self, msg):
        if self.client_connected==False:
            if msg.data==1:
                self.start_time = rospy.get_time()
                self.client_connected = True
                print('Client connected')
        if self.client_connected==True:
            if msg.data==0:
                print('Client disconnected')
                # If the full trial didn't happen, reset metrics
                if (rospy.get_time()-self.start_time)/60.0<4.5: # isn't the full trial
                    self.reset_game()
                self.client_connected = False

    def update_person(self, msg):
        self.player_x_prev = self.player_x
        self.player_y_prev = self.player_y
        self.player_x = msg.xpos
        self.player_y = msg.ypos

    def update_player_info(self, msg):
        if (msg.lives_count<self.game_lives):
            self.game_lives = msg.lives_count
            print('Game now shows player has ',self.game_lives,' self.lives at time',(rospy.get_time()-self.start_time)/60.0)

    def is_player_found(self,found,adversary_x,adversary_y,adversary_x_prev,adversary_y_prev,t):
        is_adv_close = False
        is_adv_facing = False
        is_path_on_building = True

        dist = np.sqrt((adversary_x-self.player_x)**2+(adversary_y-self.player_y)**2)
        if found==False:
            if dist<=self.disttolooselife:
                is_adv_close = True
                vec_adv_facing = [adversary_x-adversary_x_prev,adversary_y-adversary_y_prev]
                vec_adv_to_player = [self.player_x-adversary_x,self.player_y-adversary_y]
                if np.max(vec_adv_facing)>0:
                    vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                else:
                    vec_adv_facing = [100,100]
                vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                if dotproduct<(np.cos(np.pi/4)): # Can see 45 degrees right and left
                    is_adv_facing = True
                    num_tests = 50

                    v2player_step = vec_adv_to_player/num_tests;
                    prev_position = [adversary_x,adversary_y]

                    i = 0;
                    is_path_on_building = False
                    # Check the length of the vector for buildings; stop if found
                    while ((is_path_on_building == False) and (i <= num_tests)):
                        new_position = prev_position + v2player_step;

                        # Check for buildings only if you have entered a new grid space along the vector.
                        if ((np.floor(new_position[0])!=np.floor(prev_position[0])) or (np.floor(new_position[1])!=np.floor(prev_position[1]))):
                            if (self.building_array[int(np.floor(new_position[0])),int(np.floor(new_position[1]))]==1):
                                is_path_on_building = True

                        prev_position = new_position
                        i += 1
                if (is_adv_close and is_adv_facing and (is_path_on_building == False)):
                    found = True
                    self.lives -= 1
                    print('Life lost at time',(t-self.start_time)/60.0)
            else:
                if dist>self.disttoreposition:
                    found = False

            return found

if __name__ == '__main__':

    # 1. Make sure to $ rosparam set use_sim_time true
    # 2. Run this script $ rosrun darpa_dataanalysis performance_subscriber.py
    # 3. Play rosbag adding file to end $ rosbag play -r 20 --clock
    file = "/home/murpheylab/catkin_ws/src/VR_exp_ROS/darpa_dataanalysis/src/performance.csv"

    sub = 39
    control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
    con = 4
    environments = ['low','high']
    env = 1

    if sub<10:
        subID = '0' + str(sub)
    else:
        subID = str(sub)

    # Check if row had already been entered
    row_found = False
    with open(file,'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            if row[0]==subID:
                if row[1]==control[con]:
                    if row[2]==environments[env]:
                        row_found = True
                        print('Row for this trial has already been saved')
                        break
    print('Run this: rosbag play -r 35 --clock ./sub'+subID+'/'+subID+'_'+control[con]+'_'+environments[env]+'.bag')

    row_found = False
    # Listen to trial and save once finished
    if row_found==False:
        game_data = parse_bag(environments[env])
        end_time = 5*60
        end1 = ((5.0/3)*1)*60  # + 40
        end2 = ((5.0/3)*2)*60  # + 40
        end1_on = True
        end2_on = True
        done = False
        while (not rospy.is_shutdown()) and done==False:
            if game_data.start_time>0 and done==False:
                game_time = rospy.get_time()-game_data.start_time
                # print(game_data.start_time,rospy.get_time(),game_time)
                if game_time>end_time:
                    # print(game_time)
                    done = True
                # if game_time>end1 and end1_on==True:
                #     # print(game_time,end1_on)
                #     end1_treas = game_data.treasures
                #     # Append data file
                #     row = [subID,control[con],environments[env],game_data.lives,end1_treas]
                #     with open(file, 'a') as csvfile:
                #         testwriter = csv.writer(csvfile,delimiter=',')
                #         testwriter.writerow(row)
                #     print('Saved row to file: ', row)
                #     end1_on = False
                #     game_data.lives = 8
                # if game_time>end2 and end2_on:
                #     # print(game_time,end2_on)
                #     end2_treas = game_data.treasures-end1_treas
                #     # Append data file
                #     row = [subID,control[con],environments[env],game_data.lives,end2_treas]
                #     with open(file, 'a') as csvfile:
                #         testwriter = csv.writer(csvfile,delimiter=',')
                #         testwriter.writerow(row)
                #     print('Saved row to file: ', row)
                #     end2_on = False
                #     game_data.lives = 8

            rospy.sleep(.1)

        if game_data.game_lives!=game_data.lives:
            print('game_lives: ',game_data.game_lives,'lives: ',game_data.lives)
        else:
            print('lives: ',game_data.lives)

        # game_data.treasures = game_data.treasures-end1_treas-end2_treas
        # Append data file
        row = [subID,control[con],environments[env],game_data.lives,game_data.treasures]
        with open(file, 'a') as csvfile:
            testwriter = csv.writer(csvfile,delimiter=',')
            testwriter.writerow(row)
        print('Saved row to file: ', row)
