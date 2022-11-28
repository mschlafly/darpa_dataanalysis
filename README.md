# "Collaborative Robot Augment Human Cognition" Data and Statistical Analyses

A human subject experiment was conducted to test the impact of swarm control paradigm on cognitive availability and performance. This repo contains
* code for extracting raw data from rosbags and Somnomedics software
* raw data files
* code for data analysis and plotting
* plots for individual participants
* code for statistical analyses
* full, annotated statistical output files

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

## Python
save_data.py and save_data_subscriber.py were run using python version 2.7.18. The remaining python scripts use python version 3.9


## Statistical output files
The full statistical results are provided in the Stats/ folder. The filename indicates the outcome metric, the test performed (repeated measures ANOVA or generalized linear model), the subset of participants included (all, experts, or novices), and the subset of trials included (control = all 5 control paradigms; autonomy = only coverage control paradigms). Regardless of whether the primary statistical tests suggest that the control paradigm has a different effect in the low and high density environments, we included tests looking at performance in just the low/high density environments for your reference. Compareexpertise indicates only factor for expertise level (expert/novice) and control paradigm are included--these statistical tests were performed to justify the choice to separate the experts and novices. Further explanations are included in the statistical output files and in the .r scripts.

## Subjects included in plots
33 subjects have almost full datasets (at least 9 trials completed): [1, 7, 8,
9, 11, 13, 14, 15, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42] Sub15 is missing only waypoint low and sub 38 is missing only directergodic high. Sub17 and sub23 are missing heart data for sharedergodic high. Sub09 is not included in the heart analysis because of an extremely high resting heartrate. Sub19 is missing 2 bags directergodic high and sharedergodic low, and is not included in the analysis. We started collecting difficulty rating data after the study had already begun, thus Sub07, Sub08, and Sub09 do not have difficulty rating data.

* Input
  * 31 subjects with full datasets are included in input plots and analyses: ['01', '07', '08', '09', '11', '13', '14', '17', '18', '20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '30', '31', '32', '33', '34', '35', '36', '37', '39', '40', '41', '42']
  * 24 experts; 7 novices

* RR Interval
  * 29 subjects with full datasets are included in input plots and analyses: ['01', '07', '08', '09', '11', '13', '14', '18', '20', '21', '22', '24', '25', '26', '27', '28', '29', '30', '31', '32', '33', '34', '35', '36', '37', '39', '40', '41', '42']
  * 23 experts; 6 novices

* Game Score, MDP analyses
  * 33 subjects with almost full datasets are included in input plots and analyses: ['01', '07', '08', '09', '11', '13', '14', '15', '17', '18', '20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '30', '31', '32', '33', '34', '35', '36', '37', '38', '39', '40', '41', '42']
  * 25 experts; 8 novices

* Difficulty Rating
  * 30 subjects with full datasets are included in input plots and analyses: ['01', '11', '13', '14', '15', '17', '18', '20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '30', '31', '32', '33', '34', '35', '36', '37', '38', '39', '40', '41', '42']
  * 23 experts; 7 novices


## Scripts for plotting and formatting for statistical tests
Note, instructions for running specific scripts are provided at the top of each script.
  - plot_data.py reads raw_data/raw_data.csv, raw_data/RR_raw.csv, and raw_data/subject_info.csv, formats the data for statistical processing in raw_data_formatted/raw_data_formatted.csv, and plots the score, input, cognitive availability, decision-making, robot utility, and difficulty results, saving the plots to the Plots/ folder. To look at experienced and novice participants separately, edit the boolean at the top of the plot_data.py script.


## Scripts for statistical tests

Note, instructions for running specific scripts are provided at the top of each script. For each script, edit the variable skill near the top of the file to specify which participant group to analyze. 'all' includes all participants and control type, building density, and expertise level as experimental factors. 'expert' and 'novice' includes participants with >999 hours and <999 hours, respectively, of video game experience with control type and building density as experimental factors. Change the file directory at the beginning of each script.
* Input
  * stat-test-rm-inputdifficulty.r performs repeated measures ANOVAs, followed by post-hoc t-tests. Since no assumptions are violated, no additional statistical tests are required
* RR Interval
  * stat-test-rm-RR.r performs repeated measures ANOVAs, followed by post-hoc t-tests. Since no assumptions are violated, no additional statistical tests are required
* Score
  <!-- * stat-test-rm.r performs repeated measures ANOVAs. However, the normality assumption is violated--this typically leads to overly-conservative, and thus incorrect results with lower statistical power. Therefore, we used a generalized linear model as well, but included these results for reference. -->
  * stat-test-glmer.r fits a generalized linear model to the data with the experimental factors at predictors, then performs a Wald ANOVA, followed by a Tukey post-hoc test

* Difficulty rating
  * stat-test-rm-inputdifficulty.r performs repeated measures ANOVAs. However, the normality assumption is violated, and thus incorrect results with lower statistical power. Therefore, we used a generalized linear model as well, but included these results for reference.
  * SECOND TEST, ADD HERE AFTER PERFORMING


## Scripts for extracting raw data
Note, instructions for running specific scripts are provided at the top of each script.

During the experiment, data is saved using rosbags and the Somnomedics software. To begin the analysis, relevant measures need to be extracted.
  - save_data.py extracts the raw trial data as .csv files in raw_data/* by parsing rosbags. It calls scripts parse_robag.py and populate_buildings.py in the utils folder. For matching the biometric data with experimental trials, save_data.py also extracts the start and end times of each experimental trial. Some of the rosbags contain a faulty/corrupted data point; if save_data.py was unable to parse a given trial's rosbag (either because it was missing or was faulty) , the trial identifying information is printed to the .csv file unparsable_bags.
  - save_data_subscriber.py is used to parse rosbag files that are faulty listed in unparsable_bags.csv. (Note that unparsable_bags also include trials that were never run.)
  - add_difficulty_data.py adds the difficulty level rating the participant gave for that particular experimental trial. It requires the trial_info sheets specifying the randomized order of the experimental trials and difficulty_rating.csv, which has the unmatched difficulty ratings. Running this code adds a column to raw_data.csv. Directories may need to be changed at the top of the script. This script calls utils/difficulty.csv
  - save_heart.py extracts the RR interval data from Somnomedics files into raw_data/RR_raw.csv. Along the way, it saves some files in the temp_delete folder.


## System Architecture
. \
├── ... \
├── src                   # \
│   ├── Plots    # ... \
│   ├── raw_data          # ... \
│   ├── raw_data_formatted          # \
│   ├── Stats   # \
│   ├── utils   # \     
│   ├── ...     # \      
├── .gitignore \
├── ... \
└── README.md

## Authors

* Millicent Schlafly
* Katarina Popovic
* Geneva Schlafly

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md)
file for details.
