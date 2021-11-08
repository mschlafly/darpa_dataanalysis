# Run this script with python3

# This programs extracts 'RR' interval data and saves it to csv file.
# Along the way, it creates a csv with raw, unformatted data.

# Imports
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from utils.plot_utils import make_boxplot, add_stats#, make_boxplot_control
from utils.heart import *
from utils.patch import getpatchedprogress  # to clear symbols on cmd
progress = getpatchedprogress()
from progress.bar import ChargingBar

def raw_to_metricfile(sub_list, measure, my_measure):
    # if type_raw:
    #     # outputs into Indiv Heart
    #     # if need_original:
    #     get_raw_heart(sub_list, measure, my_measure)
    #     print(f"I got the raw {measure} and called it {my_measure}")
    #     # outputs into Trials
    #     # if need_trials:
    #     select_trials(sub_list, my_measure, filter_threshold)
    #     print(f"I put the {my_measure} into trials.")
    # else:
    # outputs into Indiv Heart
    get_raw_heart(sub_list, measure, my_measure, measure_type="Analyzed")
    print(f"I got the analyzed {measure} and called it {my_measure}")
    # outputs into Trials
    select_trials(sub_list, my_measure, filter_threshold, frequency_step=0)
    print(f"I put the {my_measure} into trials.")

    if need_filtering == True:
        # outputs into Trials Filtered
        filter_data(sub_list, my_measure, threshold=filter_threshold)
        print(f"I filtered the {my_measure} into Trials Filtered.")
    # if need_indiv_distribution == True:
    #     indiv_distribution(sub_list, my_measure, plot=plot_hist_scatter)
    #     print(f"I found the distibution of {my_measure} for each subject.")
        # indiv_stats(sub_list, my_measure, "ChangeDirPerc", plot=False)
        # print(f"I found the distibution of ChangeDirPerc for each subject.")
        # compute_spikes(sub_list, my_measure, plot=plot_spikes)
        # print(f"I computed and plotted the spikes of {my_measure} for each subject.")
    if need_metricfile == True:
        # outputs into Metric Files/my_measure
        combine_metrics(sub_list, my_measure, type_raw=type_raw)
        print(f"We have a metric file for {my_measure}!")

# def metricfile_to_boxplot_and_R(sub_list):
# Initialize matrices to store participant data. Matrices have rows for every
# participant even though not all have complete data. A variable sub is
# indicated the number of actual subjects with full datasets stored.
# These matrices are in the format for boxplots

    # These matrices are filled while looping through raw data
    maxsub = 43
    m0 = np.zeros((maxsub,10))
    m1 = np.zeros((maxsub,10))
    # m2 = np.zeros((maxsub,10))
    # m3 = np.zeros((maxsub,10))
    # m4 = np.zeros((maxsub,10))
    # m5 = np.zeros((maxsub,10))
    # m6 = np.zeros((maxsub,10))
    # m7 = np.zeros((maxsub,10))
    m0_list = [0] * 10
    m1_list = [0] * 10
    # m2_list = [0] * 10
    # m3_list = [0] * 10
    # m4_list = [0] * 10
    # m5_list = [0] * 10
    # m6_list = [0] * 10
    # m7_list = [0] * 10

    m_all = [m0,m1]#,m2,m3,m4,m5,m6,m7]
    m_list = [m0_list,m1_list]#,m2_list,m3_list,m4_list,m5_list,m6_list,m7_list]
    # metric_all = [np.zeros((maxsub,10))]#[0:num_metrics]
    metric_all = m_all#[0:num_metrics]
    metric_list = m_list#m_list#[0:num_metrics]

    # Recording subjects that contain all low/high complexity trials
    either = 0; only_low = 0; only_high = 0; both = 0
    low_and_high_str=''; only_low_str=''; only_high_str=''; not_low_or_high_str=''

    # String for experimental conditions
    environments = ['low','high']
    control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
    autonomy = ['direct','shared','auto']

    # file_subdata = "subject_info.csv" # contains subject info from the questionaire
    #         # including the number of hours spent playing video games over their lifetime

    # if not os.path.exists(boxplot_save_path):
    #     os.makedirs(boxplot_save_path)
    # if not os.path.exists("Stat Files/"+my_measure):
    #     os.makedirs("Stat Files/"+my_measure)

    file = "temp_delete/"+my_measure+'_raw.csv'
    df = pd.read_csv(file)
    metric_names = ['Mean']#df.columns[3:num_metrics+3].tolist() # where the we get the metric names
    # print(metric_names)

    # # Set up csvs for storing data to process in R
    # if save_data:
    file_subdata = "raw_data/subject_info.csv"
    save_file = "raw_data/"+my_measure+"_raw.csv"
    columns = ['Subject','Control','Complexity','Trial',
                'Lifetime','Expertise','Alllow','Allhigh','Mean','Zscore']
    #     # 'Uselow' and 'Usehigh' are booleans stating whether you have all of
    #     # the data for all of the low complexity trials or all of the low complexity trials
    with open(save_file,'w',newline='') as csvfile:
        testwriter = csv.writer(csvfile,delimiter=',')
        testwriter.writerow(columns)
    # # if save_data_control:
    # #     columns = ['Subject','Control','Trial','Perweek',
    # #                 'Lifetime','Expertise', *metric_names]
    # #     with open(save_file_control,'w',newline='') as csvfile:
    # #         testwriter = csv.writer(csvfile,delimiter=',')
    # #         testwriter.writerow(columns)


    subnum = 0  # to count the number of subjects included
    # Look through all participants
    for sub in sub_list:
        trial_happened = np.zeros(10)
        subID = subject_to_str(sub)

        # Collect metric data for subject
        df_sub = df[df["Subject"]==sub]
        # Saves low complexity data
        df_low = df_sub[df_sub["Complexity"]=="low"]
        for i in range(len(control)):
            df_low_control = df_low[df_low["Control"] == control[i]]
            if not df_low_control.empty:
                for j in range(len(metric_all)):
                    metric_all[j][sub,i] = df_low_control.iloc[0][j+3]
                trial_happened[i] = 1
        # Saves high complexity data
        df_high = df_sub[df_sub["Complexity"]=="high"]
        for i in range(len(control)):
            df_high_control = df_high[df_high["Control"] == control[i]]
            if not df_high_control.empty:
                for j in range(len(metric_all)):
                    metric_all[j][sub,i+5] = df_high_control.iloc[0][j+3]
                trial_happened[i+5] = 1


        # Read subject_info.csv for the amount of video games the person plays
        df_subdata = pd.read_csv(file_subdata)
        df_subrow = df_subdata[df_subdata['Subject'] == sub]
        lifetime = df_subrow.iloc[0]['Lifetime']
        perweek = df_subrow.iloc[0]['Perweek']
        skill = 0
        if lifetime > 999.0:
            expertise = "expert"
        else:
            expertise = "novice"

        # Check to see if we have data for all low complexity trials and all
        # high complexity trials seperately. Makes the assumption that no
        # trial has a zero first metric value
        if all(x != 0 for x in m0[sub,0:5]):
            all_low = 1
        else:
            all_low = 0
        if all(x != 0 for x in m0[sub,5:10]):
            all_high = 1
        else:
            all_high = 0

        # Saves data for statistical tests in R
        # if save_data:
        # Include subjects if all of the experimental trials are there
        # for either all high or all low environmental complexity.
        # if all_high==1 or all_low==1:
        for i in range(10):
            if i<5:
                con = i
                env = 0
            else:
                con = i-5
                env = 1
            row_save = ["Sub"+subID,control[con],environments[env],'trial'+str(i),
                        lifetime,expertise,all_low,all_high,
                        *[metric_all[j][sub,i] for j in range(2)]]
                        # *[metric_all[j][sub,i] for j in range(num_metrics)]]
            with open(save_file,'a',newline='') as csvfile:
                testwriter = csv.writer(csvfile,delimiter=',')
                testwriter.writerow(row_save)
        # # Saves data for statistical tests in R
        # if save_data_control:
        #     # Include subjects if all of the experimental trials are there
        #     # for either all high or all low environmental complexity.
        #     if all_high==1 and all_low==1:
        #         for i in range(5):
        #             con = i
        #             env = 0
        #             value = []
        #             for j in range(num_metrics):
        #                 avg = np.mean([metric_all[j][sub,i], metric_all[j][sub,i+5]])
        #                 value = np.append(value, avg)
        #             row_save = ["Sub"+subID,control[con],'trial'+str(i),
        #                         perweek,lifetime,expertise,*value]
        #             with open(save_file_control,'a',newline='') as csvfile:
        #                 testwriter = csv.writer(csvfile,delimiter=',')
        #                 testwriter.writerow(row_save)

        # Plot bar graph for individual subject
        # if plot_each:
        # if not os.path.exists('Plots/Indiv Plots/RR/'+my_measure):
        #     os.makedirs('Plots/Indiv Plots/RR/'+my_measure)
        # plt.figure(sub)
        # width = 0.5
        # ind = np.arange(10)
        # p1 = plt.bar(ind, m0[sub,:], width)
        # plt.ylabel('Mean')
        # plt.title(f"Mean {my_measure} for Subject {sub}")
        # labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
        # plt.xticks(ind, labels)
        # plt.savefig('Plots/Indiv Plots/'+my_measure+'/'+subID+'.png')
        # plt.close()


missing = [*range(2,7),10,12,16,19]
partial = [15,17,38]  # at least all high or all low
sub_list = [1,7,8,9,11,13,14,15,17,18,*range(20,43)]
# sub_list = [7,8,9,11,13,14,15,17,18,*range(20,23),*range(24,38),*range(39,43)]
# print(sub_list)
pulse = [1,7,8,11,13,14,15,17,18,*range(20,43)]  # w/o 9
SpO2_wolow = list(set(sub_list)-{9,38})
SpO2_wolow2 = list(set(sub_list)-{9,13,21,38,39})
SpO2_wolow3 = list(set(sub_list)-{9,13,38})
SpO2_womedlow = list(set(sub_list)-{13,21,30,38})
BP_short12 = [1,13,20,33,36,38,39]
LF_short60 = [7,20,21,22,23,25,26,28,35,38]  # same for HF
# 27, 40, 21

if not os.path.exists('temp_delete/'):
    os.makedirs('temp_delete/')
sort_gametimes(sub_list)

type_raw = False  # Treats data set as being from the raw or analyzed set
my_measure = "RR"  # Choose measure name to analyze

############ raw_to_metricfile Parameters ######################################
# get_metric_file = True
need_original = True  # Extracts measure data from original form
need_trials = True  # Separates measure data into trials with trial name
need_filtering = True  # Removes measure in [0,'A', (127)]
filter_threshold = 210
need_indiv_distribution = False  # Gathers mean and stdev for each subject
# Plots are only relevant if need_indiv_distribution is True
plot_hist_scatter = False  # Plots individual distribution of measure as a
                    # histogram and a scatter plot over time across all trials
plot_spikes = False

need_metricfile = True  # Creates metric file with mean, median, zscore, etc.
orig_measure = measure_dict[my_measure][0]  # Uses measure name on original file
################################################################################
# if get_metric_file:

# if not os.path.exists('temp_delete/'):
#     os.makedirs('temp_delete/')
# if not os.path.exists('temp_delete/Trials/'):
#     os.makedirs('temp_delete/Trials/')
# if not os.path.exists('temp_delete/Trials Filtered/'):
    # os.makedirs('temp_delete/Trials Filtered/')
raw_to_metricfile(sub_list, orig_measure, my_measure)
#
# ########### metricfile_to_boxplot_and_R Parameters #############################
# get_stats = True   # saves data, plots indivual metric means, and creates boxplots
# get_stats_all = False   # saves data, plots indivual metric means, and
#                         #   creates boxplot for all, experts, and novices
#
# # file = "Metric Files/"+my_measure+"/all.csv"
# # file = "raw_data/"+my_measure+"_raw.csv"
# # file_description = my_measure  # for boxplot title and save name
# # boxplot_save_path = "Boxplots/"+my_measure+'/wo9_neg/'
# # save_file = "raw_data_formatted/"+my_measure+".csv"
# # save_file_control = "Stat Files/"+my_measure+"/control.csv"
# # file_subdata = "raw_data/subject_info.csv"
#
# save_data = True  # Saves control comparison data into csv files for statistical processing
# save_data_control = False  # Saves control comparison data with avg of complexity levels
# plot_each = True  # Creates and saves a plot for each participant
#
# # Booleans for analyzing subsets of the data
# # only_experts = False  # Only plots experts
# # only_novices = False  # Only plots novices
# # boxplot = False  # Creates and saves boxplots for sall of the participants used
# # boxplot_control = False  # Boxplots with low and high combined
# # scatter = False  # Incomplete
# # detail_labels = False # Includes title and detailed y_label on boxplots
# # negative_zscore = False # Uses negative of z-score values
#
# # Number of metrics to be used, should be no more than number of extra columns
# # if type_raw:
# #     num_metrics = 8
# # else:
# #     num_metrics = 7
# ################################################################################
# subjects = list(set(sub_list)-{9})
# # if get_stats:
#     # metricfile_to_boxplot_and_R(subjects)
# # if get_stats_all:
# #     only_experts = True; only_novices = False
# #     metricfile_to_boxplot_and_R(subjects)
# #     only_experts = False; only_novices = True
# #     metricfile_to_boxplot_and_R(subjects)
# #     only_experts = False; only_novices = False; plot_each = False
# #     metricfile_to_boxplot_and_R(subjects)
