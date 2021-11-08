# "Shared Coverage Control For Human-Swarm Collaboration Under Pressure" Data and Statistical Analyses

A human subject experiment was conducted to test the impact of swarm control paradigm on cognitive availability and performance. This repo contains
* code for extracting raw data from rosbags and Somnomedics software
* raw data files
* code for data analysis and plotting
* plots for individual participants ????
* code for statistical analyses
* full, annotated statistical output files

## System Architecture
. \
├── ... \
├── src                   # \
│   ├── aggregateplots    # ... \
│   ├── hst_info          # ... \
│   ├── hst_data          # \
│   ├── individualplots   # \
│   ├── stattests         # Results from statistical analysis performed in R \
│   ├── trial_info        # Experimental trial info for each subject \
│   ├── vis_data          # Relevant trial data for visualization in .csv format  \
│   ├── vis_figures       # Figures of each trial broken by time instance\
│   ├── vis_mp4           # Videos of trial visualization\
│   └── ...               # ... \
├── .gitignore \
├── ... \
└── README.md

## Getting Started

These instructions outline all the software that one needs to have installed on
the local machine to be able to run this code and reproduce the results.

## Robot Operating System (ROS)

The latest version this code was run on [ROS melodic](http://wiki.ros.org/melodic).

## R-Software

R software is a programming language that allows statistical analysis. For
running the code in R, you will need to install the following:
* **R software:** This can be done with a one line command from a terminal window.
  * 'sudo apt-get install r-base'
* **R-Studio:** Make sure to download the latest version from their
[download page](https://rstudio.com/products/rstudio/download/).
Once you have the .deb package make sure to
  * navigate to your download folder 'cd Downloads',
  * unpack the folder, 'sudo dpkg -i <the package name you downladed>' and
  * install the R-Studio with the following command, 'sudo apt -f install'.

### Dependencies: RMA

There's a number of **packages** that need to be installed for proper execution
of the code. After opening R software in terminal by simply typing 'R', run
following commands:
* 'install.packages("nlopt")'
  * if you get an error, try instead doing so from the root by running the
  following command in your terminal window 'sudo apt-get install libnlopt-dev'
* install.packages("lme4")
* install.packages("car")
  * if you get an error, try instead doing so from the root by running the
  following command in your terminal window 'sudo apt-get install r-cran-car'
* 'install.packages("ez")'
* 'install.paclages("rstatix")'

### Dependencies: GLM

There are additional **packages** that need to installed for running the glmer
statistical analysis. If you get an error when trying to install it directly in
RStudio, which happens in Ubuntu 18.04 OS due to available version, try
installation from a terminal window from the root code are:
* install.packages("emmeans")
 * note that there's currently no emmeans pacakge for Linux and one needs to
 install lsmeans instead and update according code in the .r scripts
* install.packages("multicomp") OR sudo apt-get install -y r-cran-multcomp
* install.packages("mvtnorm") OR 'sudo apt-get install -y r-cran-mvtnorm'

## Python 2.7.18
Dependencies:
* numpy
* rosbag
* datetime
* csv
* os
* pandas
* glob
* matplotlib


## Instructions for plotting and statistical tests
Note, instructions for running specific scripts are provided at the top of each script.

NOTE: [2, 3, 4, 5, 6, 10, 12, 16, 19, 15, 38] are the subjects with the
incomplete data with sub15 missing ONLY waypoint low and sub 38 missing ONLY
directergodic high.

***Two subjects had to come back (Sub17and 23??), so would be missing heart rate data---how is this accounted for?***
The list of subjects with a complete dataset is [1, 7, 8,
9, 11, 13, 14, 15, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
32, 33, 34, 35, 36, 37, 39, 40, 41, 42]

## Instructions for extracting raw data
Note, instructions for running specific scripts are provided at the top of each script.

During the experiment, data is saved using rosbags and the Somnomedics software. To begin the analysis, relevant measures need to be extracted.
  - save_data.py extracts the raw trial data as .csv files in raw_data/* by parsing rosbags. It calls scripts parse_robag.py and populate_buildings.py in the utils folder. For matching the biometric data with experimental trials, save_data.py also extracts the start and end times of each experimental trial. Some of the rosbags contain a faulty/corrupted data point; if save_data.py was unable to parse a given trial's rosbag (either because it was missing or was faulty) , the trial indentifying information is printed to the .csv file unparsable_bags.
    * to run,
  - save_data_subscriber.py is used to parse rosbag files that are faulty listed in unparsable_bags.csv. (Note that unparsable_bags also include trials that were never run.)
    * Update basic info within the script
      * **line 215 and 216:** file folder path location
      * **lines 220-21:** subject and trial info for the particular missing bag
    * On ROS side, make sure to
      * set use_sim_time to true via $ rosparam set use_sim_time true
      * start $ roscore
      * run this script $ rosrun darpa_data_analysis performance_subscriber.py
      * play rosbag file in a separate terminal following print instructions. NOTE:
      this assumes you are in the folder where the particular ROS bag is located.
  - add_difficulty_data.py adds the difficulty level rating the participant gave for that particular experimental trial. It requires the trial_info sheets specifying the randomized order of the experimental trials and difficulty_rating.csv, which has the unmatched difficulty ratings. Running this code adds a column to raw_data.csv. Directories may need to be changed at the top of the script. This script calls utils/difficulty.csv
  -

still left to clean up what is actually in the folders, delete commented code, make it so it doesn't print out so many files

## List of data not shared
* trial_info/*
* difficulty_rating.csv (the raw one)
* scanned human subject sheets
* heart/
* pupil data/

# Folders not pushed to github
* temp_delete *** add to github ***

### add_difficulty_data.py

### performance_subscriber.py

For extracting performance info from the faulty ROS bags (see missing_bags.csv
  file) that exist but weren't able to be processed via performance.py you will
  need to run performance_subscriber. Here are the instructions for doing so:

* Update basic info within the script
  * **line 202 and 203:** file folder path location
  * **lines 207:** subject and trial info for the particular missing bag
* On ROS side, make sure to
  * set use_sim_time to true via $ rosparam set use_sim_time true
  * start $ roscore
  * run this script $ rosrun darpa_data_analysis performance_subscriber.py
  * play rosbag file in a separate terminal following print instructions. NOTE:
  this assumes you are in the folder where the particular ROS bag is located.

### plot_data.py

This script plots figures based on data located in the hst_info and hst_data
folders. The main variables pertaining to running this code are in the following
lines:
* **line 23:** make sure that the DIR location of the files necessary for running
this code is correct
* **line 38:** change DIR location where the control.csv file and figures will be saved
* **line 49:** decide on what population of the subject, trial conditions and
corresponding metrics will be plotted based on specified boolean values

### save_data.py

Before running this code, make sure to review the following information and
make edits accordingly based on the folder structure and the analysis need:
* **line 53:** Determine DIR(ectory) location where the files will be and specify
location of the ROS bag files to be read
* **line 59:** Decide what type of data (performance and/or visualization) should be saved
* **line 86:** Specify what subjects and what trial conditions should be analyzed

### save_vis.py

This script (1) generates figures of each instance of a trial using data
generated with save_data.py script and (2) creates videos using those figures.
Before running the code, main variables that should be reviewed and updated
accordingly are:
* **line 180:** Specify desired DIR location from where to read and where to
write data
* **line 193:** Define what is being saved: figures and/or videos
* **line 200:** Determine what components of experimental trial to display
* **line 211:** Specify video settings. For more information on ffmpeg go
[here](https://hamelot.io/visualization/using-ffmpeg-to-convert-a-set-of-images-into-a-video/) .
* **line 225:** Specify what subjects and what trial instances to analyze

### stat-test-rm.r

## Built With

* [PurpleBooth](https://github.com/PurpleBooth/a-good-readme-template) - The README.md template


## Versioning

January, 2021
* **performance.py**
  * line 23, 58, 168: added /swarm_pos topic for drone trajectories
* **plot_data.py**
  * partitioned script for better readability
  * line 33, 264, 272: updated difficulty list based on the questionnaire info
* **plot_utils.py**
  * **fixed add_stats function and added spread_factor variable
* **save_data.py**
  * partitioned script for better readability
  * line 169: streamlined saving process for visualization purposes
  * line 204: added drone csv files functionality
* **save_vis.py**
  * initial upload of the script

## Authors

* Millicent Schlafly (master forked version)
* Katarina Popovic

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md)
file for details.
