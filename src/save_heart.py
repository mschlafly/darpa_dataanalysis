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
    get_raw_heart(sub_list, measure, my_measure, measure_type="Analyzed")
    print(f"I got the analyzed {measure} and called it {my_measure}")
    # outputs into Trials
    select_trials(sub_list, my_measure, filter_threshold, frequency_step=0)
    print(f"I put the {my_measure} into trials.")

    if need_filtering == True:
        # outputs into Trials Filtered
        filter_data(sub_list, my_measure, threshold=filter_threshold)
        print(f"I filtered the {my_measure} into Trials Filtered.")
    if need_metricfile == True:
        # outputs into Metric Files/my_measure
        combine_metrics(sub_list, my_measure, type_raw=type_raw)
        print(f"We have a metric file for {my_measure}!")

    # These matrices are filled while looping through raw data
    maxsub = 43
    m0 = np.zeros((maxsub,10))
    m1 = np.zeros((maxsub,10))
    m0_list = [0] * 10
    m1_list = [0] * 10

    m_all = [m0,m1]
    m_list = [m0_list,m1_list]
    metric_all = m_all
    metric_list = m_list

    # Recording subjects that contain all low/high complexity trials
    either = 0; only_low = 0; only_high = 0; both = 0
    low_and_high_str=''; only_low_str=''; only_high_str=''; not_low_or_high_str=''

    # String for experimental conditions
    environments = ['low','high']
    control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
    autonomy = ['direct','shared','auto']
    file = "temp_delete/"+my_measure+'_raw.csv'
    df = pd.read_csv(file)
    metric_names = ['Mean']

    # # Set up csvs for storing data to process in R
    file_subdata = "raw_data/subject_info.csv"
    save_file = "raw_data/"+my_measure+"_raw.csv"
    columns = ['Subject','Control','Complexity','Trial',
                'Lifetime','Expertise','Alllow','Allhigh','RR_Mean','RR_Zscore']
    #     # 'Alllow' and 'Allhigh' are booleans stating whether you have all of
    #     # the data for all of the low complexity trials or all of the low complexity trials
    with open(save_file,'w',newline='') as csvfile:
        testwriter = csv.writer(csvfile,delimiter=',')
        testwriter.writerow(columns)

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


missing = [*range(2,7),10,12,16,19]
partial = [15,17,38]  # at least all high or all low
sub_list = [1,7,8,9,11,13,14,15,17,18,*range(20,43)]
pulse = [1,7,8,11,13,14,15,17,18,*range(20,43)]  # w/o 9
SpO2_wolow = list(set(sub_list)-{9,38})
SpO2_wolow2 = list(set(sub_list)-{9,13,21,38,39})
SpO2_wolow3 = list(set(sub_list)-{9,13,38})
SpO2_womedlow = list(set(sub_list)-{13,21,30,38})
BP_short12 = [1,13,20,33,36,38,39]
LF_short60 = [7,20,21,22,23,25,26,28,35,38]  # same for HF

if not os.path.exists('temp_delete/'):
    os.makedirs('temp_delete/')
sort_gametimes(sub_list)

type_raw = False  # Treats data set as being from the raw or analyzed set
my_measure = "RR"  # Choose measure name to analyze

############ raw_to_metricfile Parameters ######################################
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
raw_to_metricfile(sub_list, orig_measure, my_measure)
