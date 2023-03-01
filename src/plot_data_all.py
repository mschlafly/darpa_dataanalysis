# This program plots overall performance metrics_perf. It also
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
import seaborn as sns


###############################################################################
# Folder location from where to get necessary hst data
###############################################################################

# raw_data cotaines overall performance, input count, and difficulty rating
file_perf = "raw_data/raw_data.csv"
metrics_perf = ['Lives', 'Treasure', 'Input', 'Score']
file_diff = "raw_data/raw_data.csv"
metrics_diff = ['Difficulty']
file_RR = "raw_data/RR_raw.csv"
metrics_RR = ['RR_Mean','RR_Zscore']
file_MDP = "raw_data/raw_data_MDP_6a_10000s.csv"
metrics_MDP = ['MDP_r']
file_POMDP_obs = "raw_data/raw_data_POMDP_obs_i_6a_10000s.csv"
metrics_POMDP_obs = ['P_switch','norm_mean','norm_bestpath','norm_userpath']
file_POMDP_obs_cum = "raw_data/raw_data_POMDP_obs_cum_6a_10000s.csv"
metrics_POMDP_obs_cum = ['P_switch_cum','norm_cum','norm_bestpath_cum','norm_userpath_cum']
file_POMDP_d = "raw_data/raw_data_POMDP_d_6a_10000s.csv"
file_N_obs = "raw_data/raw_data_n_observations.csv"
metrics_N_obs = ['N_obs']



# contains subject info from the questionaire like the number of hours spent
# playing video games over their lifetime
file_subdata = "raw_data/subject_info.csv"

###############################################################################
# Folder location where to save files
###############################################################################

# Location of the folder to save control csv file for stat analysis
file_control = "raw_data_formatted/raw_data_formatted.csv"
file_pomdo_d_new = "raw_data_formatted/raw_data_formatted_POMDP_d.csv"

# Location of folders where to save the plots
file_plot = 'Plots/'

###############################################################################
# Set booleans that determine what instances of data to plot
###############################################################################

save_data = False  # Saves data into csv file for statistical processing
plot_each = False  # Creates and saves a plot for each participant

# Booleans for analyzing subset of participants/trial
only_experts = True  # Only plots experts
# only_experts = False  # Only plots experts
only_novices = True  # Only plots novices
only_novices = False  # Only plots novices
combine_environments = True # Whether to combine the two environments
# combine_environments = False # Whether to combine the two environments

# Check for errors
if only_experts and only_novices:
    print("Error: only one of these can be true, only_experts or only_novices")
    sfdsf
if (not only_experts) and (not only_novices):
    print("Error: combine_environments overrided and set to true")
    combine_environments = True # No need to seperate experts and novices across environments

# Booleans for plottings specific metrics_perf
plot_input = False
plot_RR = False
plot_score = False
plot_difficulty = True
plot_N_obs = False
plot_MDP = False
plot_POMDP_d = False
plot_POMDP_obs = False
# plot_RR_input_scatter = False
# plot_RR_POMDP_d_scatter = False
# if plot_RR_input_scatter and plot_RR_POMDP_d_scatter:
#     print('ERROR: cannot plot RR_input_scatter and RR_POMDP_d_scatter at same run')
#     error

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
    columns = ['Subject', 'Control', 'Density', 'Trial',
               'Perweek', 'Lifetime', 'Expertise']
    columns.extend(metrics_perf)
    columns.extend(metrics_diff)
    columns.extend(['RR_mean','RR_Zscore'])
    columns.extend(metrics_MDP)
    columns.extend(metrics_POMDP_obs)
    columns.extend(metrics_POMDP_obs_cum)
    columns.extend(metrics_N_obs)
    # columns.extend(['N_obs'])
    columns.extend(['Include_Input','Include_Score','Include_Difficulty',
    # 'Include_RR','Include_MDP_r','Include_POMDP_d','Include_N_obs'])
    # 'Include_RR','Include_MDP_r','Include_N_obs'])
    'Include_RR_mean','Include_MDP_r','Include_P_switch','Include_P_switch_cum','Include_N_obs'])
    print(columns)

    # type of control
    with open(file_control, 'w') as csvfile:
        testwriter = csv.writer(csvfile, delimiter=',')
        testwriter.writerow(columns)

# String for experimental conditions
environments = ['low', 'high']
control = ['none', 'waypoint', 'directergodic', 'sharedergodic', 'autoergodic']

# Keep track of subject IDs (with complete data) included in figures
sub_list_perf = []
sub_list_input = []
sub_list_diff = []
sub_list_RR = []
sub_list_MDP = []
sub_list_POMDP_obs = []
sub_list_POMDP_obs_cum = []
# sub_list_POMDP_d = []
sub_list_N_obs = []


data_list_perf = []
for met in metrics_perf:
    data_list_perf.append([0] * 10)
data_list_input = []
for met in metrics_perf:
    data_list_input.append([0] * 10)
data_list_diff = []
for met in metrics_diff:
    data_list_diff.append([0] * 10)
data_list_RR = []
for met in metrics_RR:
    data_list_RR.append([0] * 10)
data_list_MDP = []
for met in metrics_MDP:
    data_list_MDP.append([0] * 10)
data_list_POMDP_obs = []
for met in metrics_POMDP_obs:
    data_list_POMDP_obs.append([0] * 10)
data_list_POMDP_obs_cum = []
for met in metrics_POMDP_obs_cum:
    data_list_POMDP_obs_cum.append([0] * 10)
# data_list_POMDP_d = []
# for met in metrics_POMDP_d:
#     data_list_POMDP_d.append([0] * 10)
data_list_N_obs = []
for met in metrics_N_obs:
    data_list_N_obs.append([0] * 10)

# if plot_RR_input_scatter:
#     figure_size = (4, 3.25)
#     fig, ax = plt.subplots(figsize=figure_size, dpi=300)
# if plot_RR_POMDP_d_scatter:
#     figure_size = (4, 3.25)
#     fig, ax = plt.subplots(figsize=figure_size, dpi=300)

# Look through all participants
maxsub += 1
for sub in range(minsub, maxsub):

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

        include_sub_in_plots = True
        if only_experts:
            if expertise == "novice":
                include_sub_in_plots = False
        if only_novices:
            if expertise == "expert":
                include_sub_in_plots = False

        # Parse Performance data and add to lists
        data_mat_perf = parse_file(file_perf, metrics_perf, sub, environments, control, plot_each)
        include_perf = use_sub(9, data_mat_perf[:,-1])
        if include_perf and include_sub_in_plots:
            data_list_perf = add_sub_to_lists(data_list_perf, data_mat_perf)
            sub_list_perf.append(subID)
        include_input = use_sub(10, data_mat_perf[:,-1])
        if include_input and include_sub_in_plots:
            data_list_input = add_sub_to_lists(data_list_input, data_mat_perf)
            sub_list_input.append(subID)

        # Parse difficulty data and add to lists
        data_mat_diff = parse_file(file_diff, metrics_diff, sub, environments, control, plot_each)
        include_diff = use_sub(10, data_mat_diff[:,-1])
        if include_diff and include_sub_in_plots:
            data_list_diff = add_sub_to_lists(data_list_diff, data_mat_diff)
            sub_list_diff.append(subID)

        # Parse cog load data and add to lists
        data_mat_RR = parse_file(file_RR, metrics_RR, 'Sub'+subID, environments, control, plot_each)
        include_RR = use_sub(10, data_mat_RR[:,-1])
        if include_RR and include_sub_in_plots:
            data_list_RR = add_sub_to_lists(data_list_RR, data_mat_RR)
            sub_list_RR.append(subID)

        # Parse MDP data and add to lists
        data_mat_MDP = parse_file(file_MDP, metrics_MDP, sub, environments, control, plot_each)
        include_MDP = use_sub(8, data_mat_MDP[:,-1])
        if include_MDP and include_sub_in_plots:
            data_list_MDP = add_sub_to_lists(data_list_MDP, data_mat_MDP)
            sub_list_MDP.append(subID)

        # Parse POMDP_d data and add to lists
        # data_mat_POMDP_d = parse_file(file_POMDP_d, metrics_POMDP_d, sub, environments, control, plot_each)
        # include_POMDP_d = use_sub(7, data_mat_POMDP_d[:,-1])
        # if include_POMDP_d and include_sub_in_plots:
            # data_list_POMDP_d = add_sub_to_lists(data_list_POMDP_d, data_mat_POMDP_d)
            # sub_list_POMDP_d.append(subID)

        # Parse POMDP_obs data and add to lists
        data_mat_POMDP_obs = parse_file(file_POMDP_obs, metrics_POMDP_obs, sub, environments, control, plot_each)
        include_POMDP_obs = use_sub(2, data_mat_POMDP_obs[:,-1])
        if include_POMDP_obs and include_sub_in_plots:
            data_list_POMDP_obs = add_sub_to_lists(data_list_POMDP_obs, data_mat_POMDP_obs)
            sub_list_POMDP_obs.append(subID)

        data_mat_POMDP_obs_cum = parse_file(file_POMDP_obs_cum, metrics_POMDP_obs_cum, sub, environments, control, plot_each)
        include_POMDP_obs_cum = use_sub(6, data_mat_POMDP_obs_cum[:,-1])
        if include_POMDP_obs and include_sub_in_plots:
            data_list_POMDP_obs_cum = add_sub_to_lists(data_list_POMDP_obs_cum, data_mat_POMDP_obs_cum)
            sub_list_POMDP_obs_cum.append(subID)

        # Parse N_obs data and add to lists
        data_mat_N_obs = parse_file(file_N_obs, metrics_N_obs, sub, environments, control, plot_each)
        include_N_obs = use_sub(8, data_mat_N_obs[:,-1]) # 8 because none-low and none-high do not have observations
        if include_N_obs and include_sub_in_plots:
            # data_list_N_obs = add_sub_to_lists(data_list_N_obs, data_mat_N_obs)
            sub_list_N_obs.append(subID)


        if save_data:
            if include_perf or include_diff or include_RR or include_MDP:
            # if np.sum(trial_happened)>=9:
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
                                    perweek, lifetime, expertise]
                        row_save.extend(return_metrics_for_trial(data_list_perf, data_mat_perf,i))
                        row_save.extend(return_metrics_for_trial(data_list_diff, data_mat_diff,i))
                        row_save.extend(return_metrics_for_trial(data_list_RR, data_mat_RR,i))
                        row_save.extend(return_metrics_for_trial(data_list_MDP, data_mat_MDP,i))
                        # row_save.extend(return_metrics_for_trial(data_list_POMDP_d, data_mat_POMDP_d,i))
                        row_save.extend(return_metrics_for_trial(data_list_POMDP_obs, data_mat_POMDP_obs,i))
                        row_save.extend(return_metrics_for_trial(data_list_POMDP_obs_cum, data_mat_POMDP_obs_cum,i))
                        row_save.extend(return_metrics_for_trial(data_list_N_obs, data_mat_N_obs,i))

                        # row_save.extend([include_input,include_perf,include_diff,include_RR,include_MDP,include_POMDP_d])
                        # row_save.extend([include_input,include_perf,include_diff,include_RR,include_MDP,include_N_obs])
                        row_save.extend([include_input,include_perf,include_diff,include_RR,include_MDP,include_POMDP_obs,include_POMDP_obs_cum,include_N_obs])
                        testwriter.writerow(row_save)


print('The number of subjects included in input plots is ' + str(len(sub_list_input)) + '.')
print('These are the subjects: ',sub_list_input)
print('The number of subjects included in performance plots is ' + str(len(sub_list_perf)) + '.')
print('These are the subjects: ',sub_list_perf)
print('The number of subjects included in cognitive load plots is ' + str(len(sub_list_RR)) + '.')
print('These are the subjects: ',sub_list_RR)
print('The number of subjects included in difficulty plots is ' + str(len(sub_list_diff)) + '.')
print('These are the subjects: ',sub_list_diff)
print('The number of subjects included in MDP plots is ' + str(len(sub_list_MDP)) + '.')
print('These are the subjects: ',sub_list_MDP)
print('The number of subjects included in N observation plots is ' + str(len(sub_list_N_obs)) + '.')
print('These are the subjects: ',sub_list_N_obs)
# print('The number of subjects included in POMDP decision quality plots is ' + str(len(sub_list_POMDP_d)) + '.')
# print('These are the subjects: ',sub_list_POMDP_d)
print('The number of subjects included in POMDP drone observation plots is ' + str(len(sub_list_POMDP_obs)) + '.')
print('These are the subjects: ',sub_list_POMDP_obs)
print('The number of subjects included in POMDP drone observation cumulative plots is ' + str(len(sub_list_POMDP_obs_cum)) + '.')
print('These are the subjects: ',sub_list_POMDP_obs_cum)

###############################################################################
# PLOT BOXPLOTS
# aplha: sets color transparency used for differentiating high and low env.
# figure_size: sets the size of the figure in inches
###############################################################################

figure_size = (4, 3.25)
figure_size_thin = (2, 2.75)

# Plot parameters: 5 items
alphas5 = [None, None, None, None, None]
colors5 = ['#BA4900','#BA0071','#0071BA','#00BA49','#3c2692ff']
colors5_light = ['#f2c4a5ff','#dd81b9ff','#7fb7dcff','#7fdca3ff','#7fdcd2ff']
labels5 = ['Baseline','Waypoint','User','Shared\nControl','Fully\nAutonomous']

# Plot parameters: 4 items
alphas4 = 1
colors4 = ['#dd81b9ff','#7fb7dcff','#7fdca3ff','#7fdcd2ff']
labels4 = ['Waypoint','User','Shared\nControl','Fully\nAutonomous']


# Plotting overall game score: bar plot
if plot_score:
    fnct_plot_score_paper(metrics_perf,data_list_perf,only_experts,only_novices,combine_environments,file_plot,labels5, colors5, alphas5,figure_size)
    fnct_plot_performance(only_experts,only_novices,combine_environments,file_plot,labels5,colors5_light,alphas5,figure_size)

# Plotting command input counts
if plot_input:
    fnct_plot_input(metrics_perf,data_list_input,only_experts,only_novices,combine_environments,file_plot,figure_size)

# Plotting trial difficulty rating
if plot_difficulty:
    fnct_plot_difficulty(metrics_diff,data_list_diff,only_experts,only_novices,combine_environments,file_plot,labels5, colors5, alphas5,figure_size)

# Plotting RR results
if plot_RR:
    fnct_plot_RR(metrics_RR,data_list_RR,only_experts,only_novices,combine_environments,file_plot, labels5, colors5, alphas5,figure_size)

# Plotting N_obs results
if plot_N_obs:
    fnct_plot_N_obs(only_experts,only_novices,combine_environments,file_plot,labels4,colors4,alphas4,figure_size_thin)

# Plotting MDP results
if plot_MDP:
    fnct_plot_MDP(metrics_MDP,data_list_MDP,only_experts,only_novices,combine_environments,file_plot, labels5, colors5, alphas5,figure_size)


if plot_POMDP_d:
    fnct_plot_POMDP_d(only_experts,only_novices,combine_environments,file_plot,file_POMDP_d,file_pomdo_d_new,figure_size,colors5_light)


if plot_POMDP_obs:
    df = pd.read_csv(file_POMDP_obs)
    for met_i in range(len(metrics_POMDP_obs)):
        metric = metrics_POMDP_obs[met_i]
        fnct_plot_POMDP_obs(df, metric, only_experts,only_novices,combine_environments,file_plot,labels4,colors4,alphas4,figure_size_thin)
    df = pd.read_csv(file_POMDP_obs_cum)
    for met_i in range(len(metrics_POMDP_obs_cum)):
        metric = metrics_POMDP_obs_cum[met_i]
        fnct_plot_POMDP_obs(df, metric, only_experts,only_novices,combine_environments,file_plot,labels4,colors4,alphas4,figure_size_thin)

plt.show()
