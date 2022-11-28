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
file_MDP = "raw_data/raw_data_MDP.csv"
metrics_MDP = ['MDP_r']
file_POMDP_obs = "raw_data/raw_data_POMDP_obs.csv"
metrics_POMDP_obs = ['P_switch','norm_mean','norm_bestpath','norm_userpath','P_switch_cum','norm_cum','norm_bestpath_cum','norm_userpath_cum']
# file_POMDP_d = "raw_data/raw_data_POMDP_d.csv"
# metrics_POMDP_d = ['Perc_regret_0','Perc_regret_1','Perc_regret_2']
# # metrics_POMDP_d = ['Perc_regret_0']
file_N_obs = "raw_data/raw_data_n_observations.csv"
metrics_N_obs = ['N_observations']



# contains subject info from the questionaire like the number of hours spent
# playing video games over their lifetime
file_subdata = "raw_data/subject_info.csv"

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

save_data = True  # Saves data into csv file for statistical processing
plot_each = False  # Creates and saves a plot for each participant

# Booleans for analyzing subset of participants
only_experts = True  # Only plots experts
only_novices = False  # Only plots novices

# Booleans for plottings specific metrics_perf
plot_input = False
plot_RR = False
plot_score = True
plot_difficulty = False
plot_MDP = False
plot_POMDP_d = False
plot_POMDP_obs = True
plot_N_obs = True
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
    columns = ['Subject', 'Control', 'Complexity', 'Trial',
               'Perweek', 'Lifetime', 'Expertise']
    columns.extend(metrics_perf)
    columns.extend(metrics_diff)
    columns.extend(metrics_RR)
    columns.extend(metrics_MDP)
    # columns.extend(metrics_POMDP_d)
    columns.extend(metrics_N_obs)
    columns.extend(['Include_Input','Include_Score','Include_Difficulty',
    # 'Include_RR','Include_MDP_r','Include_POMDP_d','Include_N_obs'])
    'Include_RR','Include_MDP_r','Include_N_obs'])
    print(columns)

    # type of control
    with open(file_control, 'w') as csvfile:
        testwriter = csv.writer(csvfile, delimiter=',')
        testwriter.writerow(columns)

# String for experimental conditions
environments = ['low', 'high']
control = ['none', 'waypoint', 'directergodic', 'sharedergodic', 'autoergodic']
autonomy = ['direct', 'shared', 'auto']


# Keep track of subject IDs (with complete data) included in figures
sub_list_perf = []
sub_list_diff = []
sub_list_RR = []
sub_list_MDP = []
sub_list_POMDP_obs = []
# sub_list_POMDP_d = []
sub_list_N_obs = []


data_list_perf = []
for met in metrics_perf:
    data_list_perf.append([0] * 10)
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
        include_input = use_sub(10, data_mat_perf[:,-1])
        if include_perf and include_sub_in_plots:
            data_list_perf = add_sub_to_lists(data_list_perf, data_mat_perf)
            sub_list_perf.append(subID)

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
        include_MDP = use_sub(10, data_mat_MDP[:,-1])
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
        include_POMDP_obs = use_sub(4, data_mat_POMDP_obs[:,-1])
        if include_POMDP_obs and include_sub_in_plots:
            data_list_POMDP_obs = add_sub_to_lists(data_list_POMDP_obs, data_mat_POMDP_obs)
            sub_list_POMDP_obs.append(subID)

        # Parse N_obs data and add to lists
        data_mat_N_obs = parse_file(file_N_obs, metrics_N_obs, sub, environments, control, plot_each)
        include_N_obs = use_sub(8, data_mat_N_obs[:,-1]) # 8 because none-low and none-high do not have observations
        if include_N_obs and include_sub_in_plots:
            # data_list_N_obs = add_sub_to_lists(data_list_N_obs, data_mat_N_obs)
            sub_list_N_obs.append(subID)

        # if plot_RR_input_scatter:
        #     RR_input_x = []
        #     RR_input_y = []
        #     for i in [1,2,3,6,7,8]:
        #         if data_mat_perf[i,-1] and data_mat_RR[i,-1]:
        #             for met_i in range(len(metrics_perf)):
        #                 if metrics_perf[met_i]=='Input':
        #                     RR_input_x.append(data_mat_perf[i,met_i])
        #             for met_i in range(len(metrics_RR)):
        #                 if metrics_RR[met_i]=='RR_Zscore':
        #                     RR_input_y.append(data_mat_RR[i,met_i])
        #     ax.scatter(RR_input_x,RR_input_y)
        #     x = np.linspace(0,30,10)
        #     a, b = np.polyfit(RR_input_x, RR_input_y, 1)
        #     ax.plot(x, a*x+b)
        # if plot_RR_POMDP_d_scatter:
        #     RR_POMDP_d_x = []
        #     RR_POMDP_d_y = []
        #     for i in range(10):
        #         if data_mat_POMDP_d[i,-1] and data_mat_RR[i,-1]:
        #             for met_i in range(len(metrics_POMDP_d)):
        #                 if metrics_POMDP_d[met_i]=='Input':
        #                     RR_POMDP_d_y.append(data_mat_POMDP_d[i,met_i])
        #             for met_i in range(len(metrics_RR)):
        #                 if metrics_RR[met_i]=='RR_Zscore':
        #                     RR_POMDP_d_x.append(data_mat_RR[i,met_i])
        #     ax.scatter(RR_POMDP_d_x,RR_POMDP_d_y)
        #     x = np.linspace(0,30,10)
        #     a, b = np.polyfit(RR_POMDP_d_x,RR_POMDP_d_y, 1)
        #     ax.plot(x, a*x+b)
        #
        #
        #     # print('plotting sub '+str(sub), expertise)
        #
        #     # for i in range(10):
        #     #     if i!=0 and i!=4 and i!=5 and i!=9:
        # # Parse POMDP drone observation data and add to lists
        # data_mat = parse_file(file_POMDP_obs, metrics_POMDP_obs, sub, environments, control, plot_each)
        # include_sub = use_sub(7, data_mat[:,-1], expertise, only_experts, only_novices)
        # if include_sub:
        #     data_list_POMDP_obs = add_sub_to_lists(data_list_POMDP_obs, data_mat)
        #     sub_list_POMDP_obs.append(subID)

        # # Parse POMDP decision data and add to lists
        # data_mat = parse_file(file_POMDP_d, metrics_POMDP_d, sub, environments, control, plot_each)
        # include_sub = use_sub(4, data_mat[:,-1], expertise, only_experts, only_novices)
        # if include_sub:
        #     data_list_POMDP_d = add_sub_to_lists(data_list_POMDP_d, data_mat)
        #     sub_list_POMDP_d.append(subID)

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
                        row_save.extend(return_metrics_for_trial(data_list_N_obs, data_mat_N_obs,i))

                        # row_save.extend([include_input,include_perf,include_diff,include_RR,include_MDP,include_POMDP_d])
                        row_save.extend([include_input,include_perf,include_diff,include_RR,include_MDP,include_N_obs])
                        testwriter.writerow(row_save)


# print('The number of subjects included in input plots is ' + str(subnum_perf_input) + '.')
# print('These are the subjects: ',sub_list_input)
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
print('The number of subjects included in POMDP drone obsservation plots is ' + str(len(sub_list_POMDP_obs)) + '.')
print('These are the subjects: ',sub_list_POMDP_obs)
# print('The number of subjects included in usefullness plots is ' + str(subnum_perf_norm) + '.')
# print('These are the subjects: ',sub_list_norm)

###############################################################################
# PLOT BOXPLOTS
# aplha: sets color transparency used for differentiating high and low env.
# figure_size: sets the size of the figure in inches
###############################################################################

figure_size = (4, 3.25)
figure_size = (2, 2.75)

# Plot parameters: 5 items
alphas5 = [None, None, None, None, None]
colors5 = ['#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
labels5 = ['Baseline','Waypoint','User','Shared\nControl','Fully\nAutonomous']

# Plot parameters: 4 items
alphas4 = .8
colors4 = ['#BA0071','#0071BA','#00BA49','#00BAA6']
labels4 = ['Waypoint','User','Shared\nControl','Fully\nAutonomous']


# Plotting overall game score: bar plot
if plot_score:
    fnct_plot_score_paper(metrics_perf,data_list_perf,only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size)
    fnct_plot_performance(only_experts,only_novices,file_plot,labels5,colors5,alphas5,figure_size)



# Plotting command input counts
if plot_input:
    fnct_plot_input(metrics_perf,data_list_perf,only_experts,only_novices,file_plot,figure_size)

# Plotting trial difficulty rating
if plot_difficulty:
    fnct_plot_difficulty(metrics_diff,data_list_diff,only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size)

# Plotting RR results
if plot_RR:
    fnct_plot_RR(metrics_RR,data_list_RR,only_experts,only_novices,file_plot, labels5, colors5, alphas5,figure_size)

# Plotting N_obs results
if plot_N_obs:
    fnct_plot_N_obs(only_experts,only_novices,file_plot,labels4,colors4,alphas4,figure_size)

if only_experts:
    if plot_POMDP_obs:
        fnct_plot_POMDP_obs(only_experts,only_novices,file_plot,labels4,colors4,alphas4,figure_size)


    # Plotting MDP results
    if plot_MDP:
        for met_i in range(len(metrics_MDP)):
            metric = metrics_MDP[met_i]
            plot_metric(metric,data_list_MDP[met_i],only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size)

    # # Plotting POMDP results
    # if plot_POMDP_obs:
    #     for met_i in range(len(metrics_POMDP_obs)):
    #         metric = metrics_POMDP_obs[met_i]
    #         plot_metric(metric,data_list_POMDP_obs[met_i],only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size)




    # if plot_POMDP_d:
    #     fnct_plot_POMDP_d(metrics_POMDP_d,data_list_POMDP_d,only_experts,only_novices,file_plot)
        # for met_i in range(len(metrics_POMDP_d)):
        #     metric = metrics_POMDP_d[met_i]
        #     plot_metric(metric,data_list_POMDP_d[met_i],only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size)

    # if only_experts:
    #     fig.savefig(file_plot + 'Inputs/inputs_experts.pdf')
    # elif only_novices:
    #     fig.savefig(file_plot + 'Inputs/inputs_novices.pdf')
    # else:
    #     fig.savefig(file_plot + 'Inputs/inputs.pdf')


plt.show()

if plot_POMDP_d:
    # Location of folders where to save the plots
    file_plot = 'Plots/'

    df = pd.read_csv('raw_data_formatted/raw_data_formatted.csv')
    df = df[df['Expertise']=='expert']
    df_POMDP_d = df[df['Include_POMDP_d']==True]

    df_regret = pd.read_csv('raw_data/raw_data_POMDP_d_seaborn_perc.csv')
    # df_regret = df_regret[df_regret["Max_range"]=='.6-1']

    # Set up figure parameters
    labels5 = ['Baseline','1-Robot\nCommands','3-Robot\nCommands','Shared\nControl','Fully\nAutonomous']
    title = 'Decision Regret'
    xlabel = ''
    ylabel = 'Percent Regret Compared to Optimal Agent'

    figure_size = (7, 3.25)
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)

    # sns.barplot(data=df_regret, x="Control", y="Percent_regret",
    #             ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.25)
    sns.barplot(data=df_regret, x="Control", y="Percent_regret", hue="Max_range",
                ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.1)
    # sns.barplot(data=df_regret, x="Control", y="Percent_regret", ax=ax, errcolor='black')
    # sns.stripplot(data=df_regret, x="Control", y="Percent_regret", hue="Max_range", ax=ax, size=3) #jitter=.2,
    # sns.swarmplot(data=df_regret, x="Control", y="Percent_regret", hue="Max_range", color='black', ax=ax, size=1.75) #jitter=.2,

    # x-ticks x-axis

    ax.get_legend().remove()

    # Add titles and labels
    plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
    plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
    plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    ax.set_xticklabels(labels5, fontname="sans-serif", fontsize=9)
    for label in (ax.get_yticklabels()):
        label.set_fontsize(8)
    for tick in ax.get_xticklabels():
        tick.set_rotation(30)

    labels = ['0-0.5','0.5-1','>1']
    fig.legend(labels,loc="center right")

    ax.grid(True, linestyle='-', which='major', axis='y', color='lightgrey',
               alpha=0.5)
    ax.set_axisbelow(True)

    # Add stastical signicant marking #########################################
    sig_matrix = np.array([
                            [2,4,0.00307],  # direct - auto
                            [1,2,0.0009],  # wp - direct
                            [0,1,0.01241], # none - wp
                            [2,3,0.07559],  # direct - shared
                            [1,3,0.0009],  # wp - shared
                            [1,4,0.0009],  # wp - auto
                            ])
    upper_data_bound = [.35,.5,.5,.465,.43]
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
    ax.set_ylim([.25,.64])

    fig.subplots_adjust(bottom=0.23,left=.1,right=.8)  # asjust white spacing

    fig.savefig(file_plot + 'Decisions/pomdo_d.pdf')

# if plot_RR_input_scatter:
#     # fig, ax = plt.subplots(figsize=figure_size, dpi=300)
#     # alphas_input = [None, None, None]
#     # colors_input = ['#BA0071','#0071BA','#00BA49']
#     # labels_input = ('Waypoint', 'User', 'Shared')
#
#     # ax.scatter(RR_input_x_wp,RR_input_y_wp,color=colors_input[0])
#     # ax.scatter(RR_input_x_dir,RR_input_y_dir,color=colors_input[1])
#     # ax.scatter(RR_input_x_shar,RR_input_y_shar,color=colors_input[2])
#     # x = np.linspace(0,30,30)
#     # a, b = np.polyfit(RR_input_x_wp, RR_input_y_wp, 1)
#     # ax.plot(x, a*x+b,color=colors_input[0])
#     # a, b = np.polyfit(RR_input_x_dir, RR_input_y_dir, 1)
#     # ax.plot(x, a*x+b,color=colors_input[1])
#     # a, b = np.polyfit(RR_input_x_shar, RR_input_y_shar, 1)
#     # ax.plot(x, a*x+b,color=colors_input[2])
#     # fig.title('')
#     plt.ylabel('Cognitive Availbility Zscore')
#     plt.xlabel('Number of Instructions Provided')
#     plt.show()
# if plot_RR_POMDP_d_scatter:
#     plt.xlabel('Cognitive Availbility Zscore')
#     plt.ylabel('Decision Quality')
#     plt.show()
