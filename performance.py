#!/usr/bin/env python

import rosbag
import numpy as np
from populate_buildings import populate_building_array

class parse_bag:

    def __init__(self,filename,env):

        self.bag = rosbag.Bag(filename)
        self.topic_list = ['/treasure_info','/adversary_1_position','/adversary_2_position','/adversary_3_position','/person_position','/player_info','/client_count']
        # self.topic_list = ['/treasure_info','/adversary_1_position','/adversary_2_position','/hi','/person_position','/player_info','/client_count']
        self.disttolooselife = 1.5
        self.disttoreposition = 8
        self.building_array = populate_building_array(env)
        self.reset_game()
        self.loop_through_topics()

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
        self.game_time = 5*60 + 60 # corresponds to the game being over
        self.client_connected = False

    def loop_through_topics(self):
        for topic, msg, t in self.bag.read_messages(topics=self.topic_list):
            if topic == self.topic_list[0]:
                if (msg.treasure_count>self.treasures):
                    self.treasures = msg.treasure_count
            elif topic == self.topic_list[1]:
                self.adversary_1_x_prev2 = self.adversary_1_x
                self.adversary_1_y_prev2 = self.adversary_1_y
                self.adversary_1_x = msg.xpos
                self.adversary_1_y = msg.ypos
                if self.client_connected==True:
                    self.found1 = self.is_player_found(self.found1, self.adversary_1_x, self.adversary_1_y, self.adversary_1_x_prev, self.adversary_1_y_prev, t)
            elif topic == self.topic_list[2]:
                self.adversary_2_x_prev = msg.xpos
                self.adversary_2_y_prev = msg.ypos
                self.adversary_2_x = msg.xpos
                self.adversary_2_y = msg.ypos
                if self.client_connected==True:
                    self.found2 = self.is_player_found(self.found2, self.adversary_2_x, self.adversary_2_y, self.adversary_2_x_prev, self.adversary_2_y_prev, t)
            elif topic == self.topic_list[3]:
                self.adversary_3_x_prev2 = self.adversary_3_x
                self.adversary_3_y_prev2 = self.adversary_3_y
                self.adversary_3_x = msg.xpos
                self.adversary_3_y = msg.ypos
                if self.client_connected==True:
                    self.found3 = self.is_player_found(self.found3, self.adversary_3_x, self.adversary_3_y, self.adversary_3_x_prev, self.adversary_3_y_prev, t)
            elif topic == self.topic_list[4]:
                self.player_x_prev = self.player_x
                self.player_y_prev = self.player_y
                self.player_x = msg.xpos
                self.player_y = msg.ypos
            elif topic == self.topic_list[5]:
                if (msg.lives_count<self.game_lives):
                    self.game_lives = msg.lives_count
                    print('Game now shows player has ',self.game_lives,' self.lives at time',(t.secs-self.game_time)/60.0)
            elif topic == self.topic_list[6]:
                if self.client_connected==False:
                    if msg.data==1:
                        self.game_time = t.secs
                        self.client_connected = True
                if self.client_connected==True:
                    if msg.data==0:
                        # If the full trial didn't happen, reset metrics
                        if (t.secs-self.game_time)/60.0<4.5: # isn't the full trial
                            self.reset_game()
                        self.client_connected = False

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
                    print('Life lost at time',(t.secs-self.game_time)/60.0)
            else:
                if dist>self.disttoreposition:
                    found = False

            return found
