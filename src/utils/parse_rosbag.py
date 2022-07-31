#!/usr/bin/env python

import rosbag
import numpy as np
from populate_buildings import populate_building_array

class parse_bag:
    """Analyze ROS bags directly to save desired performance metrics."""

    def __init__(self, filename, sub, env):
        self.bag = rosbag.Bag(filename)
        self.topic_list = ['/player_info',
                           '/treasure_info',
                           '/adversary_1_position',
                           '/adversary_2_position',
                           '/adversary_3_position',
                           '/object_position',
                           '/person_position',
                           '/swarm_pos',
                           '/input',
                           '/obj_dist',
                           '/visual_dist',
                           '/target_distribution',
                           '/client_count']
        self.disttolooselife = 1.5
        self.disttoreposition = 8
        if sub == 17 or sub == 18:
            self.game_length = 5*60 - 32
        elif sub == 30:
            self.game_length = 5*60 - 18
        elif sub == 38:
            self.game_length = 5*60 - 15
        else:
            self.game_length = 5*60 - 13
        self.game_complete = False
        self.building_array = populate_building_array(env)
        self.sub = sub
        self.reset_game()
        self.loop_through_topics()

    def reset_game(self):
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
        self.client_connected = False
        self.dist_array = []
        self.vis_dist_array = []
        self.obj_dist_array = []
        self.dist_time = []
        self.vis_dist_time = []
        self.obj_dist_time = []
        self.drone_time = []
        self.drone1_posX = []
        self.drone1_posY = []
        self.drone2_posX = []
        self.drone2_posY = []
        self.drone3_posX = []
        self.drone3_posY = []
        self.found1 = False
        self.found2 = False
        self.found3 = False
        self.game_lives = 8
        self.game_time = 0
        self.game_treasures = 0
        self.game_on = False
        self.input_count = 0
        self.input_droneID = []
        self.input_posX = []
        self.input_posY = []
        self.input_time = []
        self.lives = 8
        self.object_id = []
        self.object_posX = []
        self.object_posY = []
        self.object_time = []
        self.player_posX = []
        self.player_posY = []
        self.player_posTheta = []
        self.player_time = []
        self.player_x = 0
        self.player_y = 0
        self.player_theta = 0
        self.player_x_prev = 0
        self.player_y_prev = 0
        self.start_time = 0  # 5*60 + 60 # corresponds to the game being over
        self.time_connected = 0
        self.treasures = 0
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
        self.treas_loc_x = 0
        self.treas_loc_y = 0
        self.treas_loc_x_prev = 0
        self.treas_loc_y_prev = 0

    def loop_through_topics(self):
        for topic, msg, t in self.bag.read_messages(topics=self.topic_list):
            if topic == self.topic_list[0]:  # /player_info
                if (msg.lives_count < self.game_lives):
                    self.game_lives = msg.lives_count
                    # print('Game now shows player has ',self.game_lives,' self.lives at time',(t.secs-self.start_time)/60.0)
            elif topic == self.topic_list[1]:  # /treasure_info
                # if self.treas_loc_x != msg.xpos or self.treas_loc_y != msg.ypos:
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
                self.treas_time.append(t.secs-self.start_time)
            elif topic == self.topic_list[2]:  # /adversary_1_position
                if msg.xpos != self.adversary_1_x or msg.ypos != self.adversary_1_y:
                    self.adversary_1_x_prev = self.adversary_1_x
                    self.adversary_1_y_prev = self.adversary_1_y
                    self.adversary_1_x = msg.xpos
                    self.adversary_1_y = msg.ypos
                    if self.game_on:
                        self.found1 = self.is_player_found(self.found1, self.adversary_1_x, self.adversary_1_y, self.adversary_1_x_prev, self.adversary_1_y_prev, t.secs)
                    self.adv0_posX.append(msg.xpos)  # List of xPos for animation
                    self.adv0_posY.append(msg.ypos)  # List of yPos for animation
                    self.adv0_theta.append(msg.theta)  # List of theta for animation
                    self.adv0_time.append(t.secs-self.start_time)
            elif topic == self.topic_list[3]:   # /adversary_2_position
                if msg.xpos != self.adversary_2_x or msg.ypos != self.adversary_2_y:
                    self.adversary_2_x_prev = msg.xpos
                    self.adversary_2_y_prev = msg.ypos
                    self.adversary_2_x = msg.xpos
                    self.adversary_2_y = msg.ypos
                    if self.game_on:
                        self.found2 = self.is_player_found(self.found2, self.adversary_2_x, self.adversary_2_y, self.adversary_2_x_prev, self.adversary_2_y_prev, t.secs)
                    self.adv1_posX.append(msg.xpos)  # List of xPos for animation
                    self.adv1_posY.append(msg.ypos)  # List of yPos for animation
                    self.adv1_theta.append(msg.theta)  # List of theta for animation
                    self.adv1_time.append(t.secs-self.start_time)
            elif topic == self.topic_list[4]:    # /adversary_3_position
                if msg.xpos != self.adversary_3_x or msg.ypos != self.adversary_3_y:
                    self.adversary_3_x_prev = self.adversary_3_x
                    self.adversary_3_y_prev = self.adversary_3_y
                    self.adversary_3_x = msg.xpos
                    self.adversary_3_y = msg.ypos
                    if self.game_on:
                        self.found3 = self.is_player_found(self.found3, self.adversary_3_x, self.adversary_3_y, self.adversary_3_x_prev, self.adversary_3_y_prev, t.secs)
                    self.adv2_posX.append(msg.xpos)  # List of xPos for animation
                    self.adv2_posY.append(msg.ypos)  # List of yPos for animation
                    self.adv2_theta.append(msg.theta)  # List of theta for animation
                    self.adv2_time.append(t.secs-self.start_time)
            elif topic == self.topic_list[5]:  # /object_position
                if msg.xpos != self.object_posX or msg.ypos != self.object_posY:
                    self.object_id.append(msg.id)
                    self.object_posX.append(msg.xpos)
                    self.object_posY.append(msg.ypos)
                    self.object_time.append(t.secs-self.start_time)
            elif topic == self.topic_list[6]:  # /player_position
                if msg.xpos != self.player_x or msg.ypos != self.player_y or msg.theta != self.player_theta:
                    self.player_x_prev = self.player_x
                    self.player_y_prev = self.player_y
                    self.player_x = msg.xpos
                    self.player_y = msg.ypos
                    self.player_theta = msg.theta
                    self.player_posX.append(msg.xpos)  # List of xPos for animation
                    self.player_posY.append(msg.ypos)  # List of yPos for animation
                    self.player_posTheta.append(msg.theta)
                    self.player_time.append(t.secs-self.start_time)
            elif topic == self.topic_list[7]:  # /swarm_pos
                self.drone_time.append(t.secs)
                self.drone1_posX.append(msg.poses[0].position.x)
                self.drone1_posY.append(msg.poses[0].position.y)
                self.drone2_posX.append(msg.poses[1].position.x)
                self.drone2_posY.append(msg.poses[1].position.y)
                self.drone3_posX.append(msg.poses[2].position.x)
                self.drone3_posY.append(msg.poses[2].position.y)
            elif topic == self.topic_list[8]:  # /input
                # droneID == 10 means empty entry
                # for waypoint, droneID can be 0, 1 or 2
                # for direct or shared, droneID (encompassing all) is 5
                if msg.droneID != 10:
                    # player_input = round((t.secs-self.start_time)/60.0, 2)
                    self.input_time.append(t.secs)
                    self.input_droneID.append(msg.droneID)
                    self.input_count += 1
                    xinput_array = np.asarray(msg.xinput)
                    self.input_posX.append(xinput_array)
                    yinput_array = np.asarray(msg.yinput)
                    self.input_posY.append(yinput_array)
            elif topic == self.topic_list[9]:  # /obj_dist
                self.obj_dist_time.append(t.secs)
                dist_array = np.asarray(msg.target_array)
                self.obj_dist_array.append(dist_array)
            elif topic == self.topic_list[10]:  # /visual_dist
                self.vis_dist_time.append(t.secs)
                dist_array = np.asarray(msg.target_array)
                self.vis_dist_array.append(dist_array)
            elif topic == self.topic_list[11]:  # /target_distribution
                self.dist_time.append(t.secs)
                dist_array = np.asarray(msg.target_array)
                self.dist_array.append(dist_array)
                # print('Dist is of type ', type(self.dist_array))
            elif topic == self.topic_list[12]:  # /client_count
                if self.client_connected is False:
                    if msg.data == 1:
                        self.time_connected = t.secs
                        self.client_connected = True
                if self.client_connected:
                    if msg.data == 0:
                        # If the full trial didn't happen, reset metrics
                        if (t.secs-self.start_time)/60.0 < 4:
                            self.reset_game()
                            print('Reset game')
                        self.client_connected = False

            if self.game_on:
                self.game_time = t.secs-self.start_time
                # print(self.game_time)
                if self.game_time > self.game_length:
                    print('Game end found')
                    self.game_on = False
                    self.end_time = t.secs
                    self.game_complete = True
            elif self.game_complete is False:
                if self.client_connected:  # If the unity play button has been pressed
                    if self.sub >= 15:
                        calibration_time = t.secs-self.time_connected
                    else:
                        calibration_time = 40
                    if calibration_time > 30:  # If the subject number was at least 15, at least 30s of calibration time had passed
                        if self.player_x != self.player_x_prev or self.player_y != self.player_y_prev:
                            if self.adversary_1_x != self.adversary_1_x_prev or self.adversary_1_y != self.adversary_1_y_prev:
                                print('Game starts')
                                self.game_on = True
                                self.start_time = t.secs

    def is_player_found(self, found, adversary_x, adversary_y, adversary_x_prev, adversary_y_prev, t):
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
                        new_position = prev_position + v2player_step;

                        # Check for buildings only if you have entered a new grid space along the vector.
                        if ((np.floor(new_position[0]) != np.floor(prev_position[0])) or (np.floor(new_position[1]) != np.floor(prev_position[1]))):
                            if (self.building_array[int(np.floor(new_position[0])), int(np.floor(new_position[1]))]==1):
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
