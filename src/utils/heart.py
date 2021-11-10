# Run this script with python3

import numpy as np
import pandas as pd
import csv
import os
import matplotlib.pyplot as plt
import seaborn as sns
import shutil  # to delete folders
import datetime as dt
from utils.patch import getpatchedprogress  # to clear symbols on cmd
progress = getpatchedprogress()
from progress.bar import ChargingBar


def subject_to_str(subject):
    if subject < 10:
        sub_str = '0'+str(subject)
    else:
        sub_str = str(subject)
    return sub_str

def sort_gametimes(sub_list):
    df = pd.read_csv('raw_data/gametime.csv')
    for sub in sub_list:
        # Retrieve the rows with our subject
        df_sub = df[df['Subject'].isin([sub])]
        file = 'temp_delete/Sub'+subject_to_str(sub)+'.csv'
        df_sub.to_csv(file, mode='w', index=False)

def cleaning_to_csv(file, csv_file, my_measure, measure_type="Raw", mode='w', header=True):
    # Removes header and seperates entries with commas
    with open(file, 'r') as infile:
        data = infile.read()
    data = data.replace('.', ',').replace(':',',').replace('; ',',')
    data = data.replace(' ',',')
    data = data.splitlines()
    if measure_type == "Raw":
        data_only = data[7:]
    else:
        data_only = data[6:]
    names = "day,month,year,hour,minute,second,millisecond,"
    with open(csv_file, mode=mode) as outfile:
        if mode == 'w':
            outfile.write(names)
            outfile.write(my_measure+'\n')
        for line in data_only:
            outfile.write(line+'\n')

def get_raw_heart(sub_list, measure, my_measure, measure_type="Raw"):
    for sub in sub_list:

        if sub == 19:
            sub_str = subject_to_str(sub)
            if not os.path.exists('temp_delete/Indiv Heart/Sub'+sub_str):
                os.makedirs('temp_delete/Indiv Heart/Sub'+sub_str)
            # Part 1
            heart = 'Heart/Sub'+sub_str+'/Part1/'+measure_type+'/'+measure+' - Sub'+sub_str+'.txt'
            csv_heart = 'temp_delete/Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            cleaning_to_csv(heart, csv_heart, my_measure, measure_type=measure_type)
            print(f'I put subject {sub} Part 1 into a CSV.')
            # Part 2
            heart = 'Heart/Sub'+sub_str+'/Part2/'+measure_type+'/'+measure+' - Sub'+sub_str+'.txt'
            csv_heart = 'temp_delete/Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            cleaning_to_csv(heart, csv_heart, my_measure, measure_type=measure_type, mode='a', header=False)
            print(f'I put subject {sub} Part 2 into the same CSV.')
        else:
            sub_str = subject_to_str(sub)
            if not os.path.exists('temp_delete/Indiv Heart/Sub'+sub_str):
                os.makedirs('temp_delete/Indiv Heart/Sub'+sub_str)
            heart = 'HST_data_local/Heart/Sub'+sub_str+'/'+measure_type+'/'+measure+' - Sub'+sub_str+'.txt'
            csv_heart = 'temp_delete/Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            # Cleans data file and adds header
            # Measure columns name is my_measure
            cleaning_to_csv(heart, csv_heart, my_measure, measure_type=measure_type)
            print(f'I put subject {sub} into a CSV.')

def select_trials(sub_list, my_measure, threshold, frequency_step=0.25):
# Finds the section of the heart recording that corresponds to each trial
# Saves trials with trial type in Trials
# frequency_step is the time between recordings
# If frequency_step is zero, a time column is not added.
    bar = ChargingBar("Selecting Trials", max=len(sub_list))
    for sub in sub_list:
        sub_str = subject_to_str(sub)
        game_file = 'temp_delete/Sub'+subject_to_str(sub)+'.csv'
        df_sub = pd.read_csv(game_file)
        store_base = 'temp_delete/Trials/Sub'+sub_str+'/'+my_measure
        if not os.path.exists(store_base):
            os.makedirs(store_base)
        for trial in range(len(df_sub)):
            # When trial starts and ends
            start_day = df_sub.iloc[trial]['Start-Day']
            start_hr = df_sub.iloc[trial]['Start-Hour']
            start_min = df_sub.iloc[trial]['Start-Min']
            start_sec = df_sub.iloc[trial]['Start-Sec']
            end_day = df_sub.iloc[trial]['End-Day']
            end_hr = df_sub.iloc[trial]['End-Hour']
            end_min = df_sub.iloc[trial]['End-Min']
            end_sec = df_sub.iloc[trial]['End-Sec']
            # Control and complexity type
            control = df_sub.iloc[trial]['Control']
            complexity = df_sub.iloc[trial]['Complexity']
            trial_file = store_base+'/Sub'+sub_str+'_'+control+'_'+complexity+'.csv'
            # Read heart data
            csv_heart = 'temp_delete/Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            df_heart = pd.read_csv(csv_heart)
            days = df_heart['day'].isin([start_day])
            # print(f"Start Hour: {start_hr}, Start Min: {start_min}, Start Sec: {start_sec}")
            # print(f"End Hour: {end_hr}, End Min: {end_min}, End Sec: {end_sec}")
            if start_hr == end_hr:
                hours = df_heart['hour'].isin([start_hr])
                mins = df_heart['minute'].isin([*range(int(start_min), int(end_min)+1)])
                df_days_hrs_mins = df_heart[days & hours & mins]
                # Remove beginning and ending seconds
                if len(df_days_hrs_mins) < threshold:
                # print(sub)
                # if df_days_hrs_mins.empty:
                    df_trial = df_days_hrs_mins
                    # print('here')
                else:
                    df_trial = drop_beg_and_end(df_days_hrs_mins, 'second', \
                                            start_sec, end_sec, start_min, end_min)
            else:
                # print("Trial is on separate hours.")
                hours = df_heart['hour'].isin([*range(int(start_hr), int(end_hr)+1)])
                df_days_hrs = df_heart[days & hours]
                # Remove beginning and end minutes and seconds
                df_days_hrs_mins = drop_beg_and_end(df_days_hrs, 'minute', \
                                                start_min, end_min, start_hr, end_hr)
                df_trial = drop_beg_and_end(df_days_hrs_mins, 'second', \
                                            start_sec, end_sec, start_min, end_min)
            # print(df_trial)
            if frequency_step != 0:
                end_time = len(df_trial)*frequency_step
                time = np.arange(0, end_time, frequency_step)
                df_trial.insert(0, 'time', time)
            if df_trial.empty:
                print(f"NO DATA FOR SUBJECT {sub} TRIAL {trial}")
            else:
                df_trial.to_csv(trial_file, index=False)
                # print(f"I saved trial {trial} for subject {sub}")
                hours = df_trial["hour"].tolist()
                mins = df_trial["minute"].tolist()
                secs = df_trial["second"].tolist()
                ms = df_trial["millisecond"].tolist()
                start = [hours[0], mins[0], secs[0], ms[0]]
                end = [hours[-1], mins[-1], secs[-1], ms[-1]]
                length = time_diff(start, end)
                # if length > 300:
                #     print(f"Subject {sub} Trial {trial} is {length} seconds long.")
        bar.next()
    bar.finish()

def drop_beg_and_end(df, time, start, end, parent_start, parent_end):
# Used in select_trials() to remove first and last unwanted time period
    if time == 'second':
        parent_time = 'minute'
    elif time == 'minute':
        parent_time = 'hour'
    too_early = []
    too_late = []
    found_start = False
    row = 0
    while found_start == False:
        if df.iloc[row][time] < start and df.iloc[row][parent_time] == parent_start:
            index = df.index[row]
            too_early.append(index)
        else:
            found_start = True
        row += 1
    found_end = False
    row = -1
    while found_end == False:
        if df.iloc[row][time] > end and df.iloc[row][parent_time] == parent_end:
            index = df.index[row]
            too_late.append(index)
        else:
            found_end = True
        row -= 1
    df_middle = df.drop([*too_early, *too_late])
    return df_middle

def filter_data(sub_list, my_measure, threshold=240):
# Removes line if entry in last columns is 0 or 'A'
# Displays number of files of length less than threshold
    bar = ChargingBar("Filtering Data", max=len(sub_list))
    num_short_files = 0
    file_count = 0
    omitted_trials = []
    for sub in sub_list:
        sub_str = subject_to_str(sub)
        dir = 'temp_delete/Trials/'+'Sub'+sub_str+'/'+my_measure+'/'
        save_dir = 'temp_delete/Trials Filtered/'+'Sub'+sub_str+'/'+my_measure
        if os.path.exists(save_dir):
            shutil.rmtree(save_dir)
        os.makedirs(save_dir)
        sub_short_files = 0
        for file in os.listdir(dir):
            df = pd.read_csv(dir+file)
            if my_measure == "SpO2":
                error = df[df.columns[-1]].isin([0,127,'A'])
            else:
                error = df[df.columns[-1]].isin([0,'A'])
            df_filter = df[~error]
            # Save file only if long enough
            if len(df_filter) < threshold:
                # print(f"{file} is of length {len(df_filter)}")
                omit_trial = f"{file} is of length {len(df_filter)}"
                omitted_trials.append(omit_trial)
                num_short_files += 1
                sub_short_files += 1
            else:
                # print('saving file',file)
                save_file = save_dir+'/'+file
                df_filter.to_csv(save_file, mode='w', index=False)
            file_count += 1
        bar.next()
    bar.finish()

        # print(f"Subject {sub} has {sub_short_files} short files.")
    print("The following trials are excluded because they do not contain enough valid data.")
    for statement in omitted_trials:
        print(statement)
    print(f"Number of short files: {num_short_files}")
    print(f"Total number of files: {file_count}")

# Functions for computing metrics
def mean_metric(file):
    df = pd.read_csv(file)
    data = df[df.columns[-1]]
    mean = np.mean(data)
    return mean
def med_metric(file):
    df = pd.read_csv(file)
    data = df[df.columns[-1]]
    median = np.median(data)
    return median
def std_metric(file):
    df = pd.read_csv(file)
    data = df[df.columns[-1]]
    std = np.std(data)
    return std
def zscore_metric(critical_value, mean, stdev):
    return (critical_value - mean)/stdev

def time_diff(time0, time1):
# Calculates the number of seconds between two time points
# Input is fomatted as: [hr0, min0, sec0, ms0], [hr1, min1, sec1, ms1]
    # changing milliseconds to microseconds
    start = list(time0)
    end = list(time1)
    start[3] = start[3]*1000
    end[3] = end[3]*1000
    start = [int(x) for x in start]
    end = [int(x) for x in end]
    t0 = dt.datetime(2000, 1, 1, *start)
    t1 = dt.datetime(2000, 1, 1, *end)
    diff = t1-t0
    difference = abs(diff.total_seconds())
    return difference

def combine_metrics(sub_list, my_measure):#, type_raw=True):
# Collects the mean, median, and zscore for each trial.
# Outputs all onto one metric file.

    # Create metric file with columns names.
    dir = "temp_delete/"
    if not os.path.exists(dir):
        os.makedirs(dir)
    all_metrics = dir+my_measure+'_raw.csv'
    with open(all_metrics, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        columns = ['Subject', 'Control', 'Complexity', 'Mean','Zscore']
        writer.writerow(columns)
    bar = ChargingBar("Combining Metrics", max=len(sub_list))
    for sub in sub_list:
        sub_str = subject_to_str(sub)
        metric_path = 'temp_delete/Trials Filtered/Sub'+sub_str+'/'+my_measure+'/'
        mean_list = []
        for file in os.listdir(metric_path):
            mean_list.append(mean_metric(metric_path+file))
        mean_sub = np.mean(mean_list)
        std_sub = np.std(mean_list)
        for file in os.listdir(metric_path):
            [subject, control, complexity] \
            = [file.split('_')[0][3:5], file.split('_')[1], file.split('_')[2][:-4]]
            mean = mean_metric(metric_path+file)
            zscore = zscore_metric(mean, mean_sub, std_sub)
            basic_data = [subject, control, complexity, mean, zscore]
            data_list = [*basic_data]
            with open(all_metrics, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(data_list)
        bar.next()
    bar.finish()

measure_dict = {"pulse": ["Pulse","Beats per Minute"],
                "heartrate": ["Heart Rate Curve","Beats per Minute"],
                "SpO2": ["SpO2","Oxygen Concentration"],
                "oxygen": ["SpO2","Oxygen Concentration"],  # analyzed version
                "systolic": ["Systolic", "Millimeters of Mercury (mmHg)"],
                "diastolic": ["Diastolic", "Millimeters of Mercury (mmHg)"],
                "MAP": ["MAP", "mmHg"],
                "LF": ["HRV LF", "ms x0.1 RMS"],
                "HF": ["HRV HF", "ms x0.1 RMS"],
                "SVB": ["SVB", "LF/HF x10"],
                "RR": ["RR-Interval", "Milliseconds"]
                }
