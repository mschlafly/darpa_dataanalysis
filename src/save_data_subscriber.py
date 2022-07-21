#!/usr/bin/env python

# This script parses ROS bags using the rosbag play option for the trials
# that were unable to be read using parse_rosbag.py script. The new info will be
# saved at the end of the raw_data.csv file. NOTE: Full list of these
# instances can be found in missing_bags.csv file. Given that the ROS bag for
# the particular experimental trial exists, detailed step by step instructions
# on running this code can be found below:

# * Update basic info within the script
#   * **line 215 and 216:** file folder path location
#   * **lines 220-21:** subject and trial info for the particular missing bag
# * On ROS side, make sure to
#   * set use_sim_time to true via $ rosparam set use_sim_time true
#   * start $ roscore
#   * run this script $ rosrun darpa_dataanalysis save_data_subscriber.py
#   * play rosbag file in a separate terminal following print instructions. NOTE:
#   this assumes you are in the folder where the particular ROS bag is located.

# import rosbag
import rospy
import numpy as np
import csv
import os
from utils.populate_buildings import populate_building_array
from user_input.msg import treasure_info, person_position, player_info, object_position, input_array
# from user_input.msg import input_array
# from user_input.msg import object_position
from std_msgs.msg import Int32
from datetime import datetime


class parse_bag:
    """Replay of ROS bags to save desired performance metrics."""

    def __init__(self, env):
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
        rospy.Subscriber('/input', input_array, self.update_input)
        rospy.Subscriber('/object_position', object_position, self.update_object)

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
        self.player_theta = 0
        self.player_x_prev = 30
        self.player_y_prev = 30
        self.found1 = False
        self.found2 = False
        self.found3 = False
        self.treas_loc_x = 0
        self.treas_loc_y = 0
        self.treas_loc_x_prev = 0
        self.treas_loc_y_prev = 0
        self.treasures = 0
        self.game_treasures = 0
        self.start_time = 0
        self.time_connected = 0
        self.client_connected = False
        self.game_on = False
        self.input_count = 0

        # Playback items
        self.object_id = []
        self.object_posX = []
        self.object_posY = []
        self.object_time = []
        self.player_posX = []
        self.player_posY = []
        self.player_posTheta = []
        self.player_time = []
        self.treas_posX = []
        self.treas_posY = []
        self.treas_time = []
        self.adv0_posX = []
        self.adv0_posY = []
        self.adv0_theta = []
        self.adv0_time = []
        self.adv1_posX = []
        self.adv1_posY = []
        self.adv1_theta = []
        self.adv1_time = []
        self.adv2_posX = []
        self.adv2_posY = []
        self.adv2_theta = []
        self.adv2_time = []

    def update_treasure(self, msg):
        if self.treas_loc_x != msg.xpos or self.treas_loc_y != msg.ypos:
            self.treas_loc_x_prev = self.treas_loc_x
            self.treas_loc_y_prev = self.treas_loc_y
            self.treas_loc_x = msg.xpos
            self.treas_loc_y = msg.ypos
            if self.game_on:
                if self.treas_loc_x_prev != self.treas_loc_x or self.treas_loc_y_prev != self.treas_loc_y:
                    self.treasures += 1
                if (msg.treasure_count > self.game_treasures):
                    self.game_treasures = msg.treasure_count
                self.treas_posX.append(msg.xpos)  # List of xPos for animation
                self.treas_posY.append(msg.ypos)  # List of yPos for animation
                self.treas_time.append(rospy.get_time()-self.start_time)

    def update_adv_1(self, msg):
        if msg.xpos != self.adversary_1_x or msg.ypos != self.adversary_1_y:
            self.adversary_1_x_prev = self.adversary_1_x
            self.adversary_1_y_prev = self.adversary_1_y
            self.adversary_1_x = msg.xpos
            self.adversary_1_y = msg.ypos
            if self.game_on:
                t = rospy.get_time()
                self.found1 = self.is_player_found(self.found1, self.adversary_1_x, self.adversary_1_y, self.adversary_1_x_prev, self.adversary_1_y_prev, t)
                self.adv0_posX.append(msg.xpos)  # List of xPos for animation
                self.adv0_posY.append(msg.ypos)  # List of yPos for animation
                self.adv0_theta.append(msg.theta)  # List of theta for animation
                self.adv0_time.append(rospy.get_time()-self.start_time)

    def update_adv_2(self, msg):
        if msg.xpos != self.adversary_2_x or msg.ypos != self.adversary_2_y:
            self.adversary_2_x_prev = msg.xpos
            self.adversary_2_y_prev = msg.ypos
            self.adversary_2_x = msg.xpos
            self.adversary_2_y = msg.ypos
            if self.game_on:
                t = rospy.get_time()
                self.found2 = self.is_player_found(self.found2, self.adversary_2_x, self.adversary_2_y, self.adversary_2_x_prev, self.adversary_2_y_prev, t)
                self.adv1_posX.append(msg.xpos)  # List of xPos for animation
                self.adv1_posY.append(msg.ypos)  # List of yPos for animation
                self.adv1_theta.append(msg.theta)  # List of theta for animation
                self.adv1_time.append(rospy.get_time()-self.start_time)

    def update_adv_3(self, msg):
        if msg.xpos != self.adversary_3_x or msg.ypos != self.adversary_3_y:
            self.adversary_3_x_prev = self.adversary_3_x
            self.adversary_3_y_prev = self.adversary_3_y
            self.adversary_3_x = msg.xpos
            self.adversary_3_y = msg.ypos
            if self.game_on:
                t = rospy.get_time()
                self.found3 = self.is_player_found(self.found3, self.adversary_3_x, self.adversary_3_y, self.adversary_3_x_prev, self.adversary_3_y_prev, t)
                self.adv2_posX.append(msg.xpos)  # List of xPos for animation
                self.adv2_posY.append(msg.ypos)  # List of yPos for animation
                self.adv2_theta.append(msg.theta)  # List of theta for animation
                self.adv2_time.append(rospy.get_time()-self.start_time)

    def update_person(self, msg):
        if msg.xpos != self.player_x or msg.ypos != self.player_y or msg.theta != self.player_theta:
            # print('updating person position')
            self.player_x_prev = self.player_x
            self.player_y_prev = self.player_y
            self.player_x = msg.xpos
            self.player_y = msg.ypos
            self.player_theta = msg.theta
            if self.game_on:
                self.player_posX.append(msg.xpos)  # List of xPos for animation
                self.player_posY.append(msg.ypos)  # List of yPos for animation
                self.player_posTheta.append(msg.theta)
                self.player_time.append(rospy.get_time()-self.start_time)

    def update_player_info(self, msg):
        if (msg.lives_count < self.game_lives):
            self.game_lives = msg.lives_count
            # print('Game now shows player has ',self.game_lives,' self.lives at time',(rospy.get_time()-self.start_time)/60.0)

    def update_client(self, msg):
        if self.client_connected is False:
            if msg.data == 1:
                # print('Unity connected')
                self.time_connected = rospy.get_time()
                self.client_connected = True
        if self.client_connected is True:
            if msg.data == 0:
                # print('Unity disconnected')
                # If the full trial didn't happen, reset metrics
                if (rospy.get_time()-self.start_time)/60.0 < 4:
                    self.reset_game()
                    print('Reset game')
                self.client_connected = False

    def update_input(self, msg):
        """ Saves number of input commands by subject."""
        if msg.droneID != 10:
            # player_input = round((rospy.get_time()-self.start_time)/60.0, 2)
            # self.time_player_input.append(player_input)
            # self.drone_ID.append(msg.droneID)
            self.input_count += 1

    def update_object(self, msg):
        if msg.xpos != self.object_posX or msg.ypos != self.object_posY:
            if self.game_on:
                self.object_id.append(msg.id)
                self.object_posX.append(msg.xpos)
                self.object_posY.append(msg.ypos)
                self.object_time.append(rospy.get_time()-self.start_time)

    def is_player_found(self, found, adversary_x, adversary_y, adversary_x_prev, adversary_y_prev,t):
        is_adv_close = False
        is_adv_facing = False
        is_path_on_building = True

        dist = np.sqrt((adversary_x-self.player_x)**2+(adversary_y-self.player_y)**2)
        if found is False:
            if dist <= self.disttolooselife:
                is_adv_close = True
                vec_adv_facing = [adversary_x-adversary_x_prev, adversary_y-adversary_y_prev]
                vec_adv_to_player = [self.player_x-adversary_x, self.player_y-adversary_y]
                if np.max(vec_adv_facing) > 0:
                    vec_adv_facing /= np.linalg.norm(vec_adv_facing)
                else:
                    vec_adv_facing = [100, 100]
                vec_adv_to_player /= np.linalg.norm(vec_adv_to_player)
                dotproduct = vec_adv_facing[0]*vec_adv_to_player[0] + vec_adv_facing[1]*vec_adv_to_player[1]
                if dotproduct < (np.cos(np.pi/4)):  # Can see 45 degrees right and left
                    is_adv_facing = True
                    num_tests = 50

                    v2player_step = vec_adv_to_player/num_tests
                    prev_position = [adversary_x, adversary_y]

                    i = 0
                    is_path_on_building = False
                    # Check the length of the vector for buildings; stop if found
                    while ((is_path_on_building is False) and (i <= num_tests)):
                        new_position = prev_position + v2player_step

                        # Check for buildings only if you have entered a new grid space along the vector.
                        if ((np.floor(new_position[0]) != np.floor(prev_position[0])) or (np.floor(new_position[1]) != np.floor(prev_position[1]))):
                            if (self.building_array[int(np.floor(new_position[0])), int(np.floor(new_position[1]))] == 1):
                                is_path_on_building = True

                        prev_position = new_position
                        i += 1
                if (is_adv_close and is_adv_facing and (is_path_on_building is False)):
                    found = True
                    self.lives -= 1
                    print('Life lost at time', (t-self.start_time)/60.0)
            else:
                if dist > self.disttoreposition:
                    found = False

            return found


###############################################################################
# Functions for csv file manipulation
###############################################################################

def create_csv(filePath, columns):
    # creates a csv file with given column names
    with open(filePath, 'w') as csvFile:
        writer = csv.writer(csvFile, delimiter=',')
        writer.writerow(columns)

def write_csv_rows(filePath, rows):
    with open(filePath, 'a') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for i in range(1,rows.shape[0]): # skip the first row
            writer.writerow(rows[i,:])

###############################################################################
# Main Function
###############################################################################
if __name__ == '__main__':

    # update the specific location of the raw_data and gametime csv files
    # that correspond to the filepath on your computer
    file = '/home/murpheylab/catkin_ws/src/darpa_dataanalysis/src/raw_data/raw_data.csv'
    file_game = '/home/murpheylab/catkin_ws/src/darpa_dataanalysis/src/raw_data/gametime.csv'
    file_playback = '/home/murpheylab/catkin_ws/src/darpa_dataanalysis/src/raw_data/playback/'

    # Update this info based on the particular ROS bag trial to be parsed. NOTE
    # that all the relavant info is stored in a missing_bags.csv file.
    sub = 5  # subject number
    con = 1   # corresponds to the control type
    env = 0   # corresponds to the enviornmental complexity

    control = ['none', 'waypoint', 'directergodic', 'sharedergodic', 'autoergodic']
    environments = ['low', 'high']

    if sub < 10:
        subID = '0' + str(sub)
    else:
        subID = str(sub)

    # Check if row had already been entered
    row_found = False
    with open(file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            if row[0] == subID:
                if row[1] == control[con]:
                    if row[2] == environments[env]:
                        row_found = True
                        print('Row for this trial has already been saved')
                        break

    # Run the command in a separate terminal to play the specific rosbag from
    # the folder where those bags are saved at. NOTE: Make sure you have
    # (1) started roscore ahead of time and (2) rosparam set use_sim_time true.
    print('Run this: rosbag play -r 20 --clock '+'sub'+subID+'/'+subID+'_'+control[con]+'_'+environments[env]+'.bag')

    row_found = False
    # Listen to trial and save once finished
    if row_found is False:
        game_data = parse_bag(environments[env])
        if sub == 17 or sub == 18:
            game_length = 5*60 - 32
        elif sub == 30:
            game_length = 5*60 - 18
        elif sub == 38:
            game_length = 5*60 - 15
        else:
            game_length = 5*60 - 13
        done = False
        while (not rospy.is_shutdown()) and done is False:
            if game_data.game_on:
                game_time = rospy.get_time()-game_data.start_time
                if game_time > game_length:
                    print('game end found')
                    game_data.game_on = False
                    start_time = datetime.fromtimestamp(game_data.start_time)
                    end_time = datetime.fromtimestamp(rospy.get_time())
                    done = True
            elif game_data.game_on is False:
                if game_data.client_connected:  # If the unity play button has been pressed
                    if sub >= 15:
                        calibration_time = rospy.get_time()-game_data.time_connected
                    else:
                        calibration_time = 40
                    if calibration_time > 30:  # If the subject number was at least 15, at least 30s of calibration time had passed
                        if game_data.player_x != game_data.player_x_prev or game_data.player_y != game_data.player_y_prev:
                            if game_data.adversary_1_x != game_data.adversary_1_x_prev or game_data.adversary_1_y != game_data.adversary_1_y_prev:
                                print('Game starts')
                                game_data.game_on = True
                                game_data.start_time = rospy.get_time()
            rospy.sleep(.1)

        # Prints discrepency in game lives between the number shown to player and counted lives
        if game_data.game_lives != game_data.lives:
            print('Discrepency in lives-- game_lives: ', game_data.game_lives, 'lives: ', game_data.lives)
        # Prints discrepency in treasures found between the number in ros and counted treasures
        if game_data.treasures != game_data.game_treasures:
            print('Discrepency in treasure-- game_treasures: ', game_data.game_treasures, 'treasures: ', game_data.treasures)

        # # Append data files
        # row = [subID, control[con], environments[env], game_data.lives, game_data.treasures, game_data.input_count]
        # with open(file, 'a') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=',')
        #     writer.writerow(row)
        # print('Saved row to file: ', row)
        #
        # row = [subID, control[con], environments[env],
        #        start_time.month, start_time.day,
        #        start_time.hour, start_time.minute, start_time.second,
        #        end_time.month, end_time.day,
        #        end_time.hour, end_time.minute, end_time.second]
        # with open(file_game, 'a') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=',')
        #     writer.writerow(row)
        # print('Saved row to file: ', row)

        # Create and fill data files for playback
        # For playback data, subject folder if not already created
        sub_folder_DIR = file_playback + 'Sub' + subID + '/'
        if not os.path.exists(sub_folder_DIR):
            os.makedirs(sub_folder_DIR)
        trialInfo = subID + '_' + control[con] + '_' + environments[env]


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
