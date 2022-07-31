# This program plots overall performance metrics. It also
# optionally saves the data for statistical testing in R. It relies upon
# functions from make_boxplot.py for plotting data and statistical results
# entered by hand. It can optionally look at only experts or novices or plot
# various other comparisons.

# Imports
import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from utils.plot_utils import *
from utils.parse_files import *

###############################################################################
# Folder location from where to get necessary hst data
###############################################################################

# raw_data cotaines overall performance, input count, and difficulty rating
file = "raw_data/raw_data.csv"
file_RR = "raw_data/RR_raw.csv"
file_MDP = "raw_data/raw_data_MDP.csv"

# contains subject info from the questionaire like the number of hours spent
# playing video games over their lifetime
file_subdata = "raw_data/subject_info.csv"

# contains subject IDs that filled out difficulty rating questionnaire
file_difficulty = "raw_data/difficulty_rating.csv"
dfID = pd.read_csv(file_difficulty, usecols=[0]).to_numpy().flatten()

###############################################################################
# Folder location where to save files
###############################################################################

# Location of the folder to save control csv file for stat analysis
file_control = "raw_data_formatted/raw_data_formatted.csv"

# Location of folders where to save the plots
file_plot = 'Plots/'

###############################################################################
# Set booleans that determine what instances of data to plot
###############################################################################

save_data = False  # Saves data into csv file for statistical processing
plot_each = False  # Creates and saves a plot for each participant

# Booleans for analyzing subset of participants
only_experts = False  # Only plots experts
only_novices = False  # Only plots novices
if only_experts or only_novices:
    save_data = False

# Boolean for plotting specific trial conditions
combine_complexity = True # keep this as True

# Booleans for plottings specific metrics
plot_difficulty = True
plot_input = True
plot_score_paper = True
plot_RR = True
plot_MDP = True

# Indicate range of subjects to include in plots within [1,42]
minsub = 1
maxsub = 42

# Specify specific subjects to skip here.
# [2,3,4,5,6,10,12,16,19,15,38] are the subjects with incomplete data
skipped_subjects = []

###############################################################################
# Main Script
###############################################################################

# Set up csv for storing data to process in R
if save_data:
    # 'Uselow' and 'Usehigh' are booleans stating whether you have all of the
    # data for all of the low or all of the high complexity trials
    columns = ['Subject', 'Control', 'Complexity', 'Trial',
               'Perweek', 'Lifetime', 'Expertise',
               'Lives', 'Treasure', 'Input', 'Difficulty',
               'Score', 'RR','Regret-cum']

    # type of control
    with open(file_control, 'w') as csvfile:
        testwriter = csv.writer(csvfile, delimiter=',')
        testwriter.writerow(columns)

# String for experimental conditions
environments = ['low', 'high']
control = ['none', 'waypoint', 'directergodic', 'sharedergodic', 'autoergodic']
autonomy = ['direct', 'shared', 'auto']

# Initialize matrices to store participant data. Matrices have rows for every
# participant even though not all have complete data. A variable sub is
# indicated the number of actual subjects with full datasets stored. If a
# subject's dataset is not full, their row is overwritten. These matrices are
# in the format for boxplots
subnum_input = 0
subnum_performace = 0
subnum_difficulty = 0
subnum_RR = 0
subnum_MDP = 0

# Keep track of subject IDs (with complete data) included in figures
sub_list_input = []
sub_list_performance = []
sub_list_difficulty = []
sub_list_RR = []
sub_list_MDP = []

maxsub += 1

# aggregate list
lives_all_list = [0] * 10
treasure_all_list = [0] * 10
input_all_list = [0] * 10
difficulty_all_list = [0] * 10
score_all_list = [0] * 10
RR_all_list = [0] * 10
regret_all_list = [0] * 10

# Look through all participants
for sub in range(minsub, maxsub):
    # trial_happened = np.zeros(10)
    trial_happened_RR = np.zeros(10)
    trial_happened_MDP = np.zeros(10)

    # Skips subjects numbers in the skipped_subjects list
    found = False
    for i in range(len(skipped_subjects)):
        if sub == skipped_subjects[i]:
            found = True

    if found is False:
        # Get string version of subject number for labeling
        if sub < 10:
            subID = '0' + str(sub)
        else:
            subID = str(sub)

        # Read subject_info.csv for the amount of video games played
        with open(file_subdata, 'r') as csvfile:
            subdata = csv.reader(csvfile)  # ,delimiter=',')
            for row in subdata:
                if str(sub) == row[0]:
                    # skill = np.mean(lives_all[sub, :])
                    perweek = row[1]
                    lifetime = row[2]
                    if float(lifetime + '.0') > 999.0:
                        expertise = "expert"
                    else:
                        expertise = "novice"

        # Import data for parsing
        [lives, treasure, input, difficulty, score, trial_happened] = parse_performance_data(file,subID,environments,control,plot_each)
        [RR_mean, RR_zscore, trial_happened_RR] = parse_cogload_data(file_RR,subID,environments,control,plot_each)
        [regret_cum, trial_happened_MDP] = parse_MDP_data(file_MDP,sub,environments,control,plot_each)

        # Saves data for statistical tests in R if there are 9/10 experimental trials
        # if (all_high == 1 or all_low == 1):
        if save_data:
            if np.sum(trial_happened)>=9:
                with open(file_control, 'a') as csvfile:
                    testwriter = csv.writer(csvfile, delimiter=',')
                    for i in range(10):
                        if i < 5:
                            con = i
                            env = 0
                        else:
                            con = i-5
                            env = 1
                            env = 1
                        row_save = ["Sub" + subID, control[con],
                                    environments[env], 'trial' + str(i),
                                    perweek, lifetime, expertise,
                                    lives[i],
                                    treasure[i],
                                    input[i],
                                    difficulty[i],
                                    score[i],
                                    RR_mean[i], regret_cum[i]]
                        testwriter.writerow(row_save)

        # Decide whether the subject should be included, default is to skip
        # and that is overwritten if we have lives data for every trial. If
        # we are only looking at either experts or novies, the participant
        # has to additionally belong to that group

        # Input plots only include participants with data from all 10 trials,
        # but performance plots include participants with at least 9 trials

        ###############################
        # Include data in input lists
        ###############################
        include_sub = use_sub(10,trial_happened, expertise, only_experts, only_novices)
        if include_sub:
            subnum_input += 1
            sub_list_input.append(subID)
            for i in range(10):
                if trial_happened[i] == 1:
                    if isinstance(input_all_list[i], int):
                        input_all_list[i] = input[i]
                    else:
                        input_all_list[i] = np.append(input_all_list[i], input[i])

        ###############################
        # Include data in performance lists
        ###############################
        include_sub = use_sub(9,trial_happened, expertise, only_experts, only_novices)
        if include_sub:
            subnum_performace += 1
            sub_list_performance.append(subID)
            if sub in dfID:
                subnum_difficulty += 1
                sub_list_difficulty.append(subID)
            for i in range(10):
                if trial_happened[i] == 1:
                    if isinstance(lives_all_list[i], int):
                        lives_all_list[i] = lives[i]
                        treasure_all_list[i] = treasure[i]
                        score_all_list[i] = score[i]
                        if sub in dfID:
                            difficulty_all_list[i] = difficulty[i]

                    else:
                        lives_all_list[i] = np.append(lives_all_list[i], lives[i])
                        treasure_all_list[i] = np.append(treasure_all_list[i], treasure[i])
                        score_all_list[i] = np.append(score_all_list[i], score[i])
                        if sub in dfID:
                            difficulty_all_list[i] = np.append(difficulty_all_list[i], difficulty[i])

        ###############################
        # Include data in CogLoad lists
        ###############################
        include_sub = use_sub(10,trial_happened_RR, expertise, only_experts, only_novices)
        if include_sub:
            subnum_RR += 1
            sub_list_RR.append(subID)
            for i in range(10):
                if trial_happened_RR[i] == 1:
                    if isinstance(RR_all_list[i], int):
                        RR_all_list[i] = RR_zscore[i]
                    else:
                        RR_all_list[i] = np.append(RR_all_list[i], RR_zscore[i])

        ###############################
        # Include data in MDP lists
        ###############################
        include_sub = use_sub(9,trial_happened_MDP, expertise, only_experts, only_novices)
        if sub==1:
            print(include_sub,trial_happened_MDP, expertise, only_experts, only_novices)

        if include_sub:
            subnum_MDP += 1
            sub_list_MDP.append(subID)
            for i in range(10):
                if trial_happened_MDP[i] == 1:
                    if isinstance(regret_all_list[i], int):
                        regret_all_list[i] = regret_cum[i]
                    else:
                        regret_all_list[i] = np.append(regret_all_list[i], regret_cum[i])

print('The number of subjects included in input plots is ' + str(subnum_input) + '.')
print('These are the subjects: ',sub_list_input)
print('The number of subjects included in performance plots is ' + str(subnum_performace) + '.')
print('These are the subjects: ',sub_list_performance)
print('The number of subjects included in cognitive load plots is ' + str(subnum_RR) + '.')
print('These are the subjects: ',sub_list_RR)
print('The number of subjects included in difficulty plots is ' + str(subnum_difficulty) + '.')
print('These are the subjects: ',sub_list_difficulty)
print('The number of subjects included in MDP plots is ' + str(subnum_MDP) + '.')
print('These are the subjects: ',sub_list_MDP)


###############################################################################
# PLOT BOXPLOTS
# aplha: sets color transparency used for differentiating high and low env.
# figure_size: sets the size of the figure in inches
###############################################################################

# Plot parameters: command input
figure_size = (4, 3.25)
alphas_input = [None, None, None]
colors_input = ['#BA0071','#0071BA','#00BA49']
labels_input = ('Waypoint', 'User', 'Shared')

# Plot parameters: 5 items
alphas5 = [None, None, None, None, None]
colors5 = ['#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
labels5 = ['No Swarm','Waypoint\nControl','User','Shared','Autonomous']

# Plot parameters: Cog load
figure_size_RR = (4.5, 3.25)
colors_RR = ['#BA4900','#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
labels_RR = ['','No Swarm','Waypoint\nControl','User','Shared','Autonomous']

xlabel = ''

###############################################################################
# Plotting overall game score: bar plot
###############################################################################
if plot_score_paper:

    if combine_complexity:
        data_low = lives_all_list[:5]
        data_high = lives_all_list[5:]
        data_lives = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data_lives.append(data_combined)
        data_low = treasure_all_list[:5]
        data_high = treasure_all_list[5:]
        data_treas = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data_treas.append(data_combined)
    else:
        data_lives = lives_all_list
        data_treas = treasure_all_list

    if only_experts:
        title = 'Game Performance'
        sig_matrix = np.array([
                    [1, 3, 0.0189], # waypoint - sharedergodic
                    [0,0,1]])
        spread_factor = 50

    elif only_novices:
        title = 'Game Performance'
        sig_matrix = np.array([
                    [2, 4, 0.0194],  # directergodic - autoergodic
                    [1, 4, 0.0416]])  # waypoint - autoergodic
        spread_factor = 20
    else:
        title = 'Game Performance'
        sig_matrix = np.array([])
        spread_factor = 30

    ylabel = 'Final Game Score'
    data_lives2 = [i * 3 for i in data_lives]
    [fig, ax, upper_data_bound] = make_stackedbar(data_lives2,
                                                  data_treas,
                                                  title, xlabel, ylabel,
                                                  labels5, colors5, alphas5,
                                                  figure_size)

    if only_novices:
        ax.set_ylim(bottom=17, top=28.5)
        y = 15
    else:
        ax.set_ylim(bottom=18, top=28.5)
        y = 17
    add_stats(upper_data_bound, sig_matrix, ax, spread_factor=spread_factor, type='bar')

    # Add arrows on the bottom of the plot
    fig.subplots_adjust(bottom=0.18)
    text_buffer = .65

    x1 = [1.75, 0]  # start of the arrow
    x2 = [4.25, 0]  # end of the arrow
    name = ['Coverage Control', '']  # single label due to combining env. complexity
    add_labels(ax, x1, x2, y, name, text_buffer)

    if only_experts:
        plt.savefig(file_plot + 'Score/score_experts.pdf')
    elif only_novices:
        plt.savefig(file_plot + 'Score/score_novices.pdf')
    else:
        plt.savefig(file_plot + 'Score/score.pdf')

###############################################################################
# Plotting command input counts
###############################################################################
if plot_input:
    # Formatting data to be plotted ###########################################
    if combine_complexity:
        data_low = input_all_list[:5]
        data_high = input_all_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
        data = data[1:4]
    else:
        data = input_all_list

    # Add stastical significant basedon ANOVA #################################
    if only_experts:
        title = 'Instructions Required to Operate Swarm'
        sig_matrix = np.array([
                    [1, 2, 0.002745709],  # direct ergodic - shared ergodic
                    [0, 2, 0.001252248]])  # waypoint - shared ergodic
    elif only_novices:
        title = 'Instructions Required to Operate Swarm'
        sig_matrix = np.array([
                    [0, 2, 0.08780068]])  # waypoint - shared ergodic
    else:
        title = 'Instructions Required to Operate Swarm'
        sig_matrix = np.array([
                    [1, 2, 0.0008782035],  # direct ergodic - shared ergodic
                    [0, 2, 9.047607e-05]])  # waypoint - shared ergodic

    # Create a plot ###########################################################
    ylabel = 'Number of Commands During Trial'
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels_input, colors_input)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=18,type='bar')

    # Add arrows on the bottom of the plot
    fig.subplots_adjust(bottom=0.18)
    text_buffer = .3

    x1 = [1.75, 0]  # start of the arrow
    x2 = [4.25, 0]  # end of the arrow
    name = ['Coverage Control', '']  # single label due to combining env. complexity
    add_labels(ax, x1, x2, y, name, text_buffer)


    # Saving the plots ########################################################
    if only_experts:
        fig.savefig(file_plot + 'Inputs/inputs_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'Inputs/inputs_novices.pdf')
    else:
        fig.savefig(file_plot + 'Inputs/inputs.pdf')

###############################################################################
# Plotting trial difficulty rating
###############################################################################
if plot_difficulty:
    if combine_complexity:
        data_low = difficulty_all_list[:5]
        data_high = difficulty_all_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = difficulty_all_list

    if only_experts:
        title = 'Experienced Participants\' Difficulty Rating'
        sig_matrix = np.array([
                        [1, 4, 0.0278],  # waypoint - auto ergodic
                        [0, 4, 0.0113]])  # none - auto ergodic
        y = 6.25
        text_buffer = .15
    elif only_novices:
        title = 'Novice Participants\' Difficulty Rating'
        sig_matrix = np.array([
                        [1, 2, 0.0223]])  # waypoint - direct ergodic
        y = 4.9
        text_buffer = .2
    else:
        title = 'Perceived Difficulty Rating'
        sig_matrix = np.array([
                        [1, 4, 0.0166],  # waypoint - autoergodic
                        [0, 2, 0.0298],  # none - directergodic
                        [0, 3, 0.0133],  # none - sharedergodic
                        [0, 3, 0.0014]])  # none - autoergodic
        y = 6.35
        text_buffer = .15

    # Create a plot ###########################################################

    ylabel = 'Ease of Usage'
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels5, colors5)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')

    # Add *Ergodic* label and arrow on the bottom of the plot #################
    fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom

    x1 = [1.75, 0]  # start of the arrow
    x2 = [4.25, 0]  # end of the arrow
    name = ['Coverage Control', '']  # single label due to combining env. complexity

    add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################

    if only_experts:
        fig.savefig(file_plot + 'Difficulty/difficulty_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'Difficulty/difficulty_novices.pdf')
    else:
        fig.savefig(file_plot + 'Difficulty/difficulty.pdf')

###############################################################################
# Plotting RR results
###############################################################################
if plot_RR:
    if combine_complexity:
        data_low = RR_all_list[:5]
        data_high = RR_all_list[5:]
        data = [np.array([0,0,0])]

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = RR_all_list

    if only_experts:
        title = '\'RR\' Interval'
        sig_matrix = np.array([
                                [1,2,0.02533904], # none - wp
                                [2,4,0.03165469],  # wp - shared
                                [2,5,0.008010422]  # wp - auto
                                ])
        y = -1
    elif only_novices:
        title = '\'RR\' Interval'
        sig_matrix = np.array([])
        y = -1.1
    else:
        title = '\'RR\' Interval'
        sig_matrix = np.array([])
        y = -.7

    # Create a plot ##########################################################
    ylabel = 'Within-Subject Z-score'
    xlabel = ''
    fig, ax = plt.subplots(figsize=figure_size_RR,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels_RR, colors_RR)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')

    # Add *Ergodic* label and arrow on the bottom of the plot #################
    fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom

    x1 = [2.75, 0]  # start of the arrow
    x2 = [5.25, 0]  # end of the arrow
    text_buffer = .1
    name = ['Coverage Control', '']  # single label due to combining env. complexity

    add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################

    if only_experts:
        fig.savefig(file_plot + 'RR/RR_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'RR/RR_novices.pdf')
    else:
        fig.savefig(file_plot + 'RR/RR.pdf')

###############################################################################
# Plotting MDP results
###############################################################################
if plot_MDP:
    if combine_complexity:
        data_low = regret_all_list[:5]
        data_high = regret_all_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = regret_all_list

    if only_experts:
        title = 'Cummulative Regret'
    elif only_novices:
        title = 'Cummulative Regret'
    else:
        title = 'Cummulative Regret'

    # Create a plot ##########################################################
    ylabel = 'Cummulative Regret'
    xlabel = ''

    fig, ax = make_boxplot(data, title, xlabel, ylabel, labels5, colors5, alphas5, figure_size)

    # # Add stastical signicant marking #########################################
    # add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')

    # Add *Ergodic* label and arrow on the bottom of the plot #################
    fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom

    x1 = [2.75, 0]  # start of the arrow
    x2 = [5.25, 0]  # end of the arrow
    text_buffer = .1
    name = ['Coverage Control', '']  # single label due to combining env. complexity

    add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################

    if only_experts:
        fig.savefig(file_plot + 'MDP/regret_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'MDP/regret_novices.pdf')
    else:
        fig.savefig(file_plot + 'MDP/regret.pdf')
