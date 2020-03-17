# darpa_dataanalysis
data analysis for the darpa project

This repositiory has the code to analyze the rosbag files for the darpa project. Most of the rosbags can be analyzed using save_data.py. save_data.py calls the class parse_bag within performance.py and saves the data to performance.csv. Bag files that could not be parsed due to a corrupted message need to be parsed using performance_subscriber.py. Before running performance_subscriber.py, start up the roscore and set a rosparam using "$ rosparam set use_sim_time true". Then edit the file location of performance.csv and the subject trial info at the beginning of the main function. Run performance_subscriber.py first, copy the "rosbag play..." command printed in the terminal, navigate to the folder containing all he rosbag subject folders in a different tab, and run the rosbag. performance_subscriber.py appends performance.csv with that trial's information.

save_data.py - calls class to parse rosbags and saves trial data to performace.csv. 
performance.py - contains class parse_bag for getting performance information from rosbags. Calls populate_buildings.py to determine what the adversaries can and cannot see. This script will need to import rosbag. It is possible to download that repository on other operating systems, however, I was using Ubuntu with ROS installed to run this code.
populate_buildings.py - fills building array according to if the trial is 'low' complexity or 'high' complexity.
performance_subscriber.py - subscribed to topics being published by "rosbag play" and appends performace.csv with that trial's info. Requires special instructions for running-- look at the beginning of the main function.
plot_data.py - reads the performace.csv file containing all of the data, reads subject_info.csv with other info collected from participants, plots the data, and saves the data in csvs that are formatted correctly for analysis in R.
make_boxplot.py - contains function to create formatted boxplot. Called by plot_data.py
stat-test.r - performs statistical analyses on the data and saves results to output files. If you want to look at a subset of the data according to some metric, that can be changed in the beginning. There are running instruction in the beginning. It is best is ezANOVA is installed, but you can check for significance (but no assumptions) using the aov() function. I ran this using Rstudio on windows.

I have been running most of this (not stat-test.r) on a Lunix operating system with ROS and python 2.1.17.
