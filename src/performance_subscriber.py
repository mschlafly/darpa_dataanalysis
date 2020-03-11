#!/usr/bin/env python

# import rosbag
import rospy
import numpy as np
from populate_buildings import populate_building_array
from user_input.msg import treasure_info, person_position, player_info
from std_msgs.msg import Int32

class parse_bag:

    def __init__(self,env):
        rospy.init_node('data_analysis', anonymous=True)
        self.rate = rospy.Rate(1)

        self.disttolooselife = 1.5
        self.disttoreposition = 8
        self.game_over = False
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
        self.start_time = 0
        self.client_connected = False

    def update_treasure(self, msg):
        if (msg.treasure_count>self.treasures):
            self.treasures = msg.treasure_count
            print('new treasure')

    def update_adv_1(self, msg):
        self.adversary_1_x_prev2 = self.adversary_1_x
        self.adversary_1_y_prev2 = self.adversary_1_y
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
        self.adversary_3_x_prev2 = self.adversary_3_x
        self.adversary_3_y_prev2 = self.adversary_3_y
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
                    print('reset game')
                else:
                    self.game_over = True
                    print('game over')
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

    x = parse_bag('high')
    # x = parse_bag()
    # rospy.spin()
    while not rospy.is_shutdown():
        if x.game_over == True:

            print(x.game_over)
        # rospy.sleep(2)
