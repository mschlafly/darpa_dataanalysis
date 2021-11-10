# This program plots the RR interval. It also
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

# raw_data cotaines RR interal data
file = "raw_data/RR_raw.csv"

# contains subject info from the questionaire like the number of hours spent
# playing video games over their lifetime
file_subdata = "raw_data/subject_info.csv"

###############################################################################
# Folder location where to save files
###############################################################################

# Location of the folder to save control csv file for stat analysis
file_control = "raw_data_formatted/RR_raw_formatted.csv"

# Location of folders where to save the plots
file_plot_all = 'Plots/RR/'
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
combine_complexity = True

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
               'Alllow', 'Allhigh', 'RR']

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
subnum = 0

# Keep track of subject IDs (with complete data) included in figures
sub_list = []

# These matrices are filled while looping through raw data
maxsub += 1
RR_all = np.zeros((maxsub, 10))-2001
RRMean_all = np.zeros((maxsub, 10))

# aggregate list
RR_all_list = [0] * 10

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
                if row[0] == 'Sub'+subID:
                    # If the row is for a low complexity trial,
                    # store data in the first 5 columns
                    if row[2] == environments[0]:
                        for i in range(len(control)):
                            if row[1] == control[i]:
                                if float(row[8]) > 0:
                                    RR_all[sub, i] = row[9]#(1/float(row[8]))*1000*60
                                    RRMean_all[sub, i] = row[8]
                                    trial_happened[i] = 1
                    # If the row is for a high complexity trial,
                    # store data in the last 5 columns
                    else:
                        for i in range(len(control)):
                            if row[1] == control[i]:
                                if float(row[8]) > 0:
                                    RR_all[sub, i+5] = row[9]#(1/float(row[8]))*1000*60
                                    RRMean_all[sub, i+5] = row[8]
                                    trial_happened[i+5] = 1

            # Plot bar graph for individual subject
            if plot_each:
                plt.figure(sub)
                width = 0.5
                ind = np.arange(10)
                p1 = plt.bar(ind, RRMean_all[sub, :], width)
                plt.ylabel('Mean RR Interval')
                plt.title('RR Interval for Subject ' + subID)
                labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                          'HN', 'HW', 'HD', 'HS', 'HA')
                plt.xticks(ind, labels)
                plt.savefig(file_plot_ind + subID + '_RR.png')
                plt.close('all')

            # Read subject_info.csv for the amount of video games played
            with open(file_subdata, 'r') as csvfile:
                subdata = csv.reader(csvfile)  # ,delimiter=',')
                for row in subdata:
                    if str(sub) == row[0]:
                        skill = 0
                        perweek = row[1]
                        lifetime = row[2]
                        if float(lifetime + '.0') > 999.0:
                            expertise = "expert"
                        else:
                            expertise = "novice"

            # Check to see if we have data for all low complexity trials and
            # all high complexity trials seperately. Makes the assumption that
            # no trial ended with 0 lives
            if np.min(RR_all[sub, 0:5]) > -2000:
                all_low = 1
            else:
                all_low = 0

            if np.min(RR_all[sub, 5:10]) > -2000:
                all_high = 1
            else:
                all_high = 0
            # if all of the experimental trials are there
            # for either all high or all low environmental complexity
            if (all_high == 1 and all_low == 1):
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
                                        all_low, all_high, RRMean_all[sub, i]]
                            testwriter.writerow(row_save)

            # Decide whether the subject should be included, default is to skip
            # and that is overwritten if we have lives data for every trial. If
            # we are only looking at either experts or novies, the participant
            # has to additionally belong to that group
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
                subnum += 1
                sub_list.append(subID)
                for i in range(10):
                    if trial_happened[i] == 1:
                        if isinstance(RR_all_list[i], int):
                            RR_all_list[i] = RR_all[sub, i]

                        else:
                            RR_all_list[i] = np.append(RR_all_list[i], RR_all[sub, i])

print('The number of subjects included in plots is ' + str(subnum) + '.')
print('These are the subjects: ',sub_list)

###############################################################################
# PLOT BOXPLOTS
# aplha: sets color transparency used for differentiating high and low env.
# figure_size: sets the size of the figure in inches
###############################################################################

# Plot parameters: 5 items
figure_size = (4.5, 3.25)
alphas = [None, None, None, None, None, None]
colors = ['#BA4900','#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
labels = ['','No Swarm','Waypoint\nControl','User','Shared','Autonomous']

xlabel = ''

###############################################################################
# Plotting trial RR rating
###############################################################################

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
fig, ax = plt.subplots(figsize=figure_size,dpi=300)
upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels, colors)

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
    fig.savefig(file_plot_all + 'RR_experts.pdf')
elif only_novices:
    fig.savefig(file_plot_all + 'RR_novices.pdf')
else:
    fig.savefig(file_plot_all + 'RR.pdf')
