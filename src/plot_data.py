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

###############################################################################
# Folder location from where to get necessary hst data
###############################################################################

# raw_data cotaines overall performance, input count, and difficulty rating
file = "raw_data/raw_data.csv"
file_RR = "raw_data/RR_raw.csv"

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
file_plot_all = 'Plots/'
file_plot_ind = 'Plots/Indiv Plots/'

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
               'Skill', 'Perweek', 'Lifetime', 'Expertise',
               'Alllow', 'Allhigh',
               'Lives', 'Treasure', 'Input', 'Difficulty', 'Score']

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

# Keep track of subject IDs (with complete data) included in figures
sub_list_input = []
sub_list_performance = []
sub_list_difficulty = []

# These matrices are filled while looping through raw data
maxsub += 1
lives_all = np.zeros((maxsub, 10))
treasure_all = np.zeros((maxsub, 10))
score_all = np.zeros((maxsub, 10))
input_all = np.zeros((maxsub, 10))
difficulty_all = np.zeros((maxsub, 10))

# aggregate list
lives_all_list = [0] * 10
treasure_all_list = [0] * 10
input_all_list = [0] * 10
difficulty_all_list = [0] * 10
score_all_list = [0] * 10

# Look through all participants
for sub in range(minsub, maxsub):
    trial_happened = np.zeros(10)
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

        # Import data for parsing
        with open(file, 'r') as csvfile:
            data = csv.reader(csvfile, delimiter=',')

            # Loop through rows in csv with each row representing a trial.
            # If the row is for the subject of interest, act
            for row in data:
                if row[0] == subID:
                    # If the row is for a low complexity trial,
                    # store data in the first 5 columns
                    if row[2] == environments[0]:
                        for i in range(len(control)):
                            if row[1] == control[i]:
                                lives_all[sub, i] = row[3]
                                treasure_all[sub, i] = row[4]
                                input_all[sub, i] = row[5]
                                difficulty_all[sub, i] = row[6]
                                score_all[sub, i] = lives_all[sub, i]*3. + treasure_all[sub, i]
                                trial_happened[i] = 1
                    # If the row is for a high complexity trial,
                    # store data in the last 5 columns
                    else:
                        for i in range(len(control)):
                            if row[1] == control[i]:
                                lives_all[sub, i+5] = row[3]
                                treasure_all[sub, i+5] = row[4]
                                input_all[sub, i+5] = row[5]
                                difficulty_all[sub, i+5] = row[6]
                                score_all[sub, i+5] = lives_all[sub, i+5]*3. + treasure_all[sub, i+5]
                                trial_happened[i+5] = 1

            # Plot bar graph for individual subject
            if plot_each:
                plt.figure(sub)
                width = 0.5
                ind = np.arange(10)
                p1 = plt.bar(ind, lives_all[sub, :], width)
                p2 = plt.bar(ind, treasure_all[sub, :], width, bottom=lives_all[sub, :])
                plt.ylabel('Score')
                plt.title('Game Performance for Subject ' + subID)
                labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                          'HN', 'HW', 'HD', 'HS', 'HA')
                plt.xticks(ind, labels)
                plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
                plt.savefig(file_plot_ind + subID + '_performance.png')

                plt.figure(sub+1)
                width = 0.5
                ind = np.arange(10)
                p1 = plt.bar(ind, input_all[sub, :], width)
                plt.ylabel('Score')
                plt.title('Inputs for Subject ' + subID)
                labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                          'HN', 'HW', 'HD', 'HS', 'HA')
                plt.xticks(ind, labels)
                plt.savefig(file_plot_ind + subID + '_input.png')

                plt.figure(sub+2)
                width = 0.5
                ind = np.arange(10)
                p1 = plt.bar(ind, difficulty_all[sub, :], width)
                plt.ylabel('Score')
                plt.title('Difficulty for Subject ' + subID)
                labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                          'HN', 'HW', 'HD', 'HS', 'HA')
                plt.xticks(ind, labels)
                plt.savefig(file_plot_ind + subID + '_difficulty.png')
                plt.close('all')

            # Read subject_info.csv for the amount of video games played
            with open(file_subdata, 'r') as csvfile:
                subdata = csv.reader(csvfile)  # ,delimiter=',')
                for row in subdata:
                    if str(sub) == row[0]:
                        skill = np.mean(lives_all[sub, :])
                        perweek = row[1]
                        lifetime = row[2]
                        if float(lifetime + '.0') > 999.0:
                            expertise = "expert"
                        else:
                            expertise = "novice"

            # Check to see if we have data for all low complexity trials and
            # all high complexity trials seperately. Makes the assumption that
            # no trial ended with 0 lives
            if np.min(lives_all[sub, 0:5]) > 0:
                all_low = 1
            else:
                all_low = 0

            if np.min(lives_all[sub, 5:10]) > 0:
                all_high = 1
            else:
                all_high = 0

            # if all of the experimental trials are there
            # for either all high or all low environmental complexity
            if (all_high == 1 or all_low == 1):
                # Saves data for statistical tests in R
                if save_data:
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
                                        skill, perweek, lifetime, expertise,
                                        all_low, all_high, lives_all[sub, i],
                                        treasure_all[sub, i],
                                        input_all[sub, i],
                                        difficulty_all[sub, i],
                                        score_all[sub, i]]
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
            include_sub = False
            if (all_low == 1) and (all_high == 1):
                if only_experts:
                    if expertise == "expert":
                        include_sub = True
                elif only_novices:
                    if expertise == "novice":
                        include_sub = True
                else:
                    include_sub = True

            if include_sub:
                subnum_input += 1
                sub_list_input.append(subID)
                for i in range(10):
                    if trial_happened[i] == 1:
                        if isinstance(input_all_list[i], int):
                            input_all_list[i] = input_all[sub, i]
                        else:
                            input_all_list[i] = np.append(input_all_list[i], input_all[sub, i])

            ###############################
            # Include data in performance lists
            ###############################
            include_sub = False
            if (all_low == 1) or (all_high == 1):
                if only_experts:
                    if expertise == "expert":
                        include_sub = True
                elif only_novices:
                    if expertise == "novice":
                        include_sub = True
                else:
                    include_sub = True

            if include_sub:
                subnum_performace += 1
                sub_list_performance.append(subID)
                if sub in dfID:
                    subnum_difficulty += 1
                    sub_list_difficulty.append(subID)
                for i in range(10):
                    if trial_happened[i] == 1:
                        if isinstance(lives_all_list[i], int):
                            lives_all_list[i] = lives_all[sub, i]
                            treasure_all_list[i] = treasure_all[sub, i]
                            score_all_list[i] = score_all[sub, i]
                            if sub in dfID:
                                difficulty_all_list[i] = difficulty_all[sub, i]

                        else:
                            lives_all_list[i] = np.append(lives_all_list[i], lives_all[sub, i])
                            treasure_all_list[i] = np.append(treasure_all_list[i], treasure_all[sub, i])
                            score_all_list[i] = np.append(score_all_list[i], score_all[sub, i])
                            if sub in dfID:
                                difficulty_all_list[i] = np.append(difficulty_all_list[i], difficulty_all[sub, i])


print('The number of subjects included in input plots is ' + str(subnum_input) + '.')
print('These are the subjects: ',sub_list_input)
print('The number of subjects included in performance plots is ' + str(subnum_performace) + '.')
print('These are the subjects: ',sub_list_performance)
print('The number of subjects included in difficulty plots is ' + str(subnum_difficulty) + '.')
print('These are the subjects: ',sub_list_difficulty)

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
        plt.savefig(file_plot_all + 'Score/score_experts.pdf')
    elif only_novices:
        plt.savefig(file_plot_all + 'Score/score_novices.pdf')
    else:
        plt.savefig(file_plot_all + 'Score/score.pdf')


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
        fig.savefig(file_plot_all + 'Inputs/inputs_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot_all + 'Inputs/inputs_novices.pdf')
    else:
        fig.savefig(file_plot_all + 'Inputs/inputs.pdf')

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
        title = 'Percieved Difficulty Rating'
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
        fig.savefig(file_plot_all + 'Difficulty/difficulty_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot_all + 'Difficulty/difficulty_novices.pdf')
    else:
        fig.savefig(file_plot_all + 'Difficulty/difficulty.pdf')
