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
            if not os.path.exists('Indiv Heart/Sub'+sub_str):
                os.makedirs('Indiv Heart/Sub'+sub_str)
            # Part 1
            heart = 'Heart/Sub'+sub_str+'/Part1/'+measure_type+'/'+measure+' - Sub'+sub_str+'.txt'
            csv_heart = 'Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            cleaning_to_csv(heart, csv_heart, my_measure, measure_type=measure_type)
            print(f'I put subject {sub} Part 1 into a CSV.')
            # Part 2
            heart = 'Heart/Sub'+sub_str+'/Part2/'+measure_type+'/'+measure+' - Sub'+sub_str+'.txt'
            csv_heart = 'Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            cleaning_to_csv(heart, csv_heart, my_measure, measure_type=measure_type, mode='a', header=False)
            print(f'I put subject {sub} Part 2 into the same CSV.')
        else:
            sub_str = subject_to_str(sub)
            if not os.path.exists('Indiv Heart/Sub'+sub_str):
                os.makedirs('Indiv Heart/Sub'+sub_str)
            heart = 'Heart/Sub'+sub_str+'/'+measure_type+'/'+measure+' - Sub'+sub_str+'.txt'
            csv_heart = 'Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
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
        # for trial in [5]:
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
            csv_heart = 'Indiv Heart/Sub'+sub_str+'/Sub'+sub_str+'_'+my_measure+'.csv'
            df_heart = pd.read_csv(csv_heart)
            days = df_heart['day'].isin([start_day])
            # print(f"Start Hour: {start_hr}, Start Min: {start_min}, Start Sec: {start_sec}")
            # print(f"End Hour: {end_hr}, End Min: {end_min}, End Sec: {end_sec}")
            if start_hr == end_hr:
                hours = df_heart['hour'].isin([start_hr])
                mins = df_heart['minute'].isin([*range(start_min, end_min+1)])
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
                hours = df_heart['hour'].isin([*range(start_hr, end_hr+1)])
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
        # if os.path.exists(dir):
        #     shutil.rmtree(dir)
        # os.makedirs(dir)
        if os.path.exists(save_dir):
            shutil.rmtree(save_dir)
        os.makedirs(save_dir)
        sub_short_files = 0
        # print(os.listdir(dir))
        for file in os.listdir(dir):
            # print('inloop')
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

# def compute_spikes(sub_list, my_measure, plot=True):
# Collects all spikes data into a csv for each trial
# Calculates and saves individual distribution for each subject
# Saves spike plots for each subject
    # bar = ChargingBar("Computing Spikes", max=len(sub_list))
    # if not os.path.exists('Metric Distribution/'+my_measure):
    #     os.makedirs('Metric Distribution/'+my_measure)
    # dist_file = 'Metric Distribution/'+my_measure+'/NumSpikes.csv'
    # columns = ['Subject', 'Mean', 'Stdev']
    # with open(dist_file, 'w', newline='') as csvfile:
    #     writer = csv.writer(csvfile, delimiter=',')
    #     writer.writerow(columns)
    # for sub in sub_list:
    #     sub_str = subject_to_str(sub)
    #     filtered_dir = 'Trials Filtered/'+'Sub'+sub_str+'/'+my_measure + '/'
    #     spike_dir = 'Spikes/'+my_measure+'/Sub'+sub_str+'/'
    #     if not os.path.exists(spike_dir):
    #         os.makedirs(spike_dir)
    #     plot_dir = 'Spike Plots/'+my_measure+'/'
    #     if not os.path.exists(plot_dir):
    #         os.makedirs(plot_dir)
    #     lengths = []
    #     df_sub = pd.DataFrame(columns=["Time", "Data", "Zscore", "Control", "Complexity"])
    #     for file in os.listdir(filtered_dir):
    #         df = spikes_metric(filtered_dir+file, my_measure)
    #         df.to_csv(spike_dir+file, index=False)
    #         df_sub = df_sub.append(df, ignore_index=True)
    #         trial_length = len(df)
    #         lengths = np.append(lengths, trial_length)
    #     mean = np.mean(lengths)
    #     std = np.std(lengths)
    #     if std == 0:
    #         # print("I changed a zero standard deviation.")
    #         std = 0.1
    #     distribution = [sub, mean, std]
    #     with open(dist_file, 'a', newline='') as csvfile:
    #         writer = csv.writer(csvfile, delimiter=',')
    #         writer.writerow(distribution)
    #         # print(df_sub)
    #     if plot:
    #         # Plot histogram
    #         plt.hist(df_sub["Data"])
    #         plt.title(f'{my_measure} spikes across all trials for subject {sub}')
    #         plt.xlabel(measure_dict[my_measure][1])
    #         plt.ylabel("Number of Spikes")
    #         # plt.xlim(85, 105)
    #         # plt.ylim(0, 450)
    #         plt.savefig(plot_dir+sub_str+'.png')
    #         # plt.show()
    #         plt.close()
    #         # Plot scatterplot
    #         sns.scatterplot(x="Time", y="Data", hue="Complexity", data=df_sub)
    #         # plt.scatter(sub_time, sub_data)
    #         plt.savefig(plot_dir+'time_'+sub_str+'.png')
    #         plt.close()
    #     bar.next()
    # bar.finish()

 # Functions for computing metrics
def mean_metric(file):
    df = pd.read_csv(file)
    data = df[df.columns[-1]]
    mean = np.mean(data)
    # mean = np.std(data)
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
    # mean = np.std(data)
    return std
# def indiv_stats(sub_list, my_measure, metric, plot=True):
# # Computes the mean and standard deviation for the metric for each subject
# # Saves info as dist_file
    # if not os.path.exists('Metric Distribution/'+my_measure):
    #     os.makedirs('Metric Distribution/'+my_measure)
#     dist_file = 'Metric Distribution/'+my_measure+'/'+metric+'.csv'
#     columns = ['Subject', 'Mean', 'Stdev']
#     with open(dist_file, 'w', newline='') as csvfile:
#         writer = csv.writer(csvfile, delimiter=',')
#         writer.writerow(columns)
#     for sub in sub_list:
#         sub_str = subject_to_str(sub)
#         dir = 'Trials Filtered/'+'Sub'+sub_str+'/'+my_measure+'/'
#         sub_data = []
#         for file in os.listdir(dir):
#             df = pd.read_csv(dir+file)
#             data = df[df.columns[-1]]
#             if metric == "ChangeDirPerc":
#                 incr_time_pts = np.zeros(len(data))
#                 increasing = False
#                 decreasing = False
#                 decr_to_incr_count = 0
#                 incr_count = 0
#                 decr_count = 0
#                 change_dir = 0
#                 for i in range(len(data)-1):
#                     if data[i+1] > data[i]:
#                         increasing = True
#                         incr_count += 1
#                         incr_time_pts[i+1] = 1
#                         if decreasing:
#                             change_dir += 1
#                             decr_to_incr_count +=1
#                             decreasing = False
#                     elif data[i+1] < data[i]:
#                         decreasing = True
#                         decr_count += 1
#                         incr_time_pts[i+1] = -1
#                         if increasing:
#                             change_dir += 1
#                             increasing = False
#                 if incr_count !=0 or decr_count != 0:
#                     incr_count_percent = incr_count/(incr_count+decr_count)*100
#                 else:
#                     incr_count_percent = 50
#                 change_dir_perc = change_dir/len(data)*100
#                 # if change_dir_perc < 10:
#                 #     print(f"File: {file} changes {change_dir} times over {len(data)} data points.")
#                 # else:
#                 #     print(len(data))
#                 sub_data = np.append(sub_data, change_dir_perc)
#         mean = np.mean(sub_data)
#         std = np.std(sub_data)
#         # Prevent divide by zero issues
#         if std == 0:
#             # print("I changed a zero standard deviation.")
#             std = 0.1
#         distribution = [sub, mean, std]
#
#         with open(dist_file, 'a', newline='') as csvfile:
#             writer = csv.writer(csvfile, delimiter=',')
#             writer.writerow(distribution)
#
#         if plot == True:
#             # Plot histogram
#             if not os.path.exists('Metric Plots/'+my_measure+'/'+metric):
#                 os.makedirs('Metric Plots/'+my_measure+'/'+metric)
#             if metric == "ChangeDirPerc":
#                 plt.hist(sub_data, bins=[*np.arange(25,75,5)])
#             else:
#                 plt.hist(sub_data, bins=100)
#             plt.title(f'{my_measure}: {metric} across all trials for subject {sub}')
#             # plt.xlabel(measure_dict[my_measure][1])
#             # plt.xlim(85, 105)
#             # plt.ylim(0, 450)
#             plt.savefig('Metric Plots/'+my_measure+'/'+metric+'/'+sub_str+'.png')
#             # plt.show()
#             plt.close()
#             # Plot scatter plot
#             time = range(0,len(sub_data))
#             plt.scatter(time, sub_data)
#             plt.title(f'{my_measure}: {metric} across all trials for subject {sub}')
#             # plt.ylabel(measure_dict[my_measure][1])
#             plt.xlabel("Time")
#             if metric == "ChangeDirPerc":
#                 plt.xlim(0,10)
#                 plt.ylim(25,75)
#             # plt.xlim(85, 105)
#             # plt.ylim(0, 2000)
#             plt.savefig('Metric Plots/'+my_measure+'/'+metric+'/time'+sub_str+'.png')
#             # plt.show()
#             plt.close()
# def indiv_distribution(sub_list, my_measure, plot=True):
# # Computes the mean and standard deviation for the metric for each subject.
# # Saves info as dist_file.
#     bar = ChargingBar(f"Determing {my_measure} Distribution", max=len(sub_list))
#     dist_file = 'Indiv Distribution/'+my_measure+'.csv'
#     columns = ['Subject', 'Mean', 'Stdev']
#     with open(dist_file, 'w', newline='') as csvfile:
#         writer = csv.writer(csvfile, delimiter=',')
#         writer.writerow(columns)
#     for sub in sub_list:
#         sub_str = subject_to_str(sub)
#         dir = 'Trials Filtered/'+'Sub'+sub_str+'/'+my_measure+'/'
#         sub_data = []
#         for file in os.listdir(dir):
#             df = pd.read_csv(dir+file)
#             data = df[df.columns[-1]]
#             sub_data = np.append(sub_data, data)
#         # mean = np.mean(np.std(sub_data))
#         mean = np.mean(sub_data)
#         std = np.std(sub_data)
#         distribution = [sub, mean, std]
#
#         with open(dist_file, 'a', newline='') as csvfile:
#             writer = csv.writer(csvfile, delimiter=',')
#             writer.writerow(distribution)
#
#         if plot == True:
#             # Plot histogram
#             if not os.path.exists('Measure Plots/'+my_measure):
#                 os.makedirs('Measure Plots/'+my_measure)
#             if my_measure == "SpO2" or my_measure == "oxygen":
#                 plt.hist(sub_data, bins=[*np.arange(79.5,101.5,1)])
#             elif my_measure == "pulse" or my_measure == "heartrate":
#                 plt.hist(sub_data, bins=[*np.arange(30.5,150.5,4)])
#             elif my_measure in ["systolic", "diastolic", "SVB"]:
#                 plt.hist(sub_data, bins=20)
#             else:
#                 plt.hist(sub_data, bins=100)
#             plt.title(f'{my_measure} across all trials for subject {sub}')
#             plt.xlabel(measure_dict[my_measure][1])
#             # plt.xlim(85, 105)
#             plt.ylim(0, 450)
#             plt.savefig('Measure Plots/'+my_measure+'/'+sub_str+'.png')
#             # plt.show()
#             plt.close()
#             # Plot scatter plot
#             time = range(0,len(sub_data))
#             plt.scatter(time, sub_data)
#             plt.title(f'{my_measure} across all trials for subject {sub}')
#             plt.ylabel(measure_dict[my_measure][1])
#             plt.xlabel("Time")
#             # plt.xlim(85, 105)
#             # plt.ylim(0, 2000)
#             plt.savefig('Measure Plots/'+my_measure+'/time'+sub_str+'.png')
#             # plt.show()
#             plt.close()
#         bar.next()
#     bar.finish()

def zscore_metric(critical_value, mean, stdev):
    return (critical_value - mean)/stdev
# def direction_metric(file):
#     df = pd.read_csv(file)
#     data = df[df.columns[-1]]
#     time = df['time']
#     incr_time_pts = np.zeros(len(data))
#     increasing = False
#     decreasing = False
#     decr_to_incr_count = 0
#     incr_count = 0
#     decr_count = 0
#     incr_start = time[0]
#     decr_start = time[0]
#     incr_time = []
#     decr_time = []
#     change_dir = 0
#     for i in range(len(data)-1):
#         if data[i+1] > data[i]:
#             increasing = True
#             incr_count += 1
#             incr_time_pts[i+1] = 1
#             if decreasing:
#                 change_dir += 1
#                 decr_to_incr_count +=1
#                 incr_start = time[i]
#                 decr_end = time[i]
#                 decr_time = np.append(decr_time, decr_end - decr_start)
#                 decreasing = False
#         elif data[i+1] < data[i]:
#             decreasing = True
#             decr_count += 1
#             incr_time_pts[i+1] = -1
#             if increasing:
#                 change_dir += 1
#                 decr_start = time[i]
#                 incr_end = time[i]
#                 incr_time = np.append(incr_time, incr_end - incr_start)
#                 increasing = False
#     if increasing:
#         incr_end = time[i]
#         incr_time = np.append(incr_time, incr_end - incr_start)
#     elif decreasing:
#         decr_end = time[i]
#         decr_time = np.append(decr_time, decr_end - decr_start)
#     incr_total_time = sum(incr_time)
#     decr_total_time = sum(decr_time)
#     # print(incr_total_time, decr_total_time, incr_total_time+decr_total_time, time[len(time)-1]-time[0])
#     incr_count_percent = incr_count/(incr_count+decr_count)*100
#     incr_time_percent = incr_total_time/(incr_total_time+decr_total_time)*100
#     change_dir_perc = change_dir/len(data)*100
#     # plt.scatter(time, incr_time_pts)
#     # plt.show()
#     # plt.close()
#
#     return [change_dir_perc, incr_count_percent, incr_time_percent]
# # def direction_notime_metric(file):
#     df = pd.read_csv(file)
#     data = df[df.columns[-1]]
#     incr_time_pts = np.zeros(len(data))
#     increasing = False
#     decreasing = False
#     decr_to_incr_count = 0
#     incr_count = 0
#     decr_count = 0
#     change_dir = 0
#     for i in range(len(data)-1):
#         if data[i+1] > data[i]:
#             increasing = True
#             incr_count += 1
#             incr_time_pts[i+1] = 1
#             if decreasing:
#                 change_dir += 1
#                 decr_to_incr_count +=1
#                 decreasing = False
#         elif data[i+1] < data[i]:
#             decreasing = True
#             decr_count += 1
#             incr_time_pts[i+1] = -1
#             if increasing:
#                 change_dir += 1
#                 increasing = False
#     if incr_count !=0 or decr_count != 0:
#         incr_count_percent = incr_count/(incr_count+decr_count)*100
#     else:
#         incr_count_percent = 50
#     change_dir_perc = change_dir/len(data)*100
#
#     # plt.scatter(time, incr_time_pts)
#     # plt.show()
#     # plt.close()
#
#     return [change_dir_perc, incr_count_percent]
# # def spikes_metric(file, my_measure):
#     df = pd.read_csv(file)
#     data = df[df.columns[-1]]
#     mean = np.mean(data)
#     stdev = np.std(data)
#     hours = df["hour"]
#     mins = df["minute"]
#     secs = df["second"]
#     ms = df["millisecond"]
#     time = np.zeros((len(data)-1,4))
#     current_spike = []
#     spike_time = pd.DataFrame(columns=['hr','min','sec','ms'])
#     max_times = []
#     max_data = []
#     max_zscore = []
#     large = False
#     for i in range(len(data)-1):
#         time[i] = np.array([hours[i], mins[i], secs[i], ms[i]])
#         # print(time[i])
#         if data[i] > mean+1.5*stdev:
#             large = True
#             current_spike = np.append(current_spike, data[i])
#             ser = pd.Series(time[i], index=['hr','min','sec','ms'])
#             spike_time = spike_time.append(ser, ignore_index=True)
#         else:
#             if large == True:
#                 max = np.max(current_spike)
#                 index = current_spike.tolist().index(max)
#                 max_value_time = spike_time.iloc[index].tolist()
#                 zscore = zscore_metric(max, mean, stdev)
#                 time_passed = time_diff(time[0], max_value_time)
#
#                 max_times = np.append(max_times, time_passed)
#                 max_data = np.append(max_data, max)
#                 max_zscore = np.append(max_zscore, zscore)
#
#                 # resetting current spike data
#                 current_spike = []
#                 spike_time = pd.DataFrame(columns=['hr','min','sec','ms'])
#                 large = False
#
#     con = file.split('_')[1]
#     control = [con] * len(max_times)
#     complex = file.split('_')[2][:-4]
#     high_low = [complex] * len(max_times)
#
#     data0 = {"Time": max_times}
#     df0 = pd.DataFrame(data0)
#     data1 = {"Data": max_data}
#     df1 = pd.DataFrame(data1)
#     data2 = {"Zscore": max_zscore}
#     df2 = pd.DataFrame(data2)
#     data3 = {"Control": control}
#     df3 = pd.DataFrame(data3)
#     data4 = {"Complexity": high_low}
#     df4 = pd.DataFrame(data4)
#
#     df = pd.concat([df0, df1, df2, df3, df4], axis=1)
#     return df

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

    # time0[3] = time0[3]*1000
    # time1[3] = time1[3]*1000
    # time0 = [int(x) for x in time0]
    # time1 = [int(x) for x in time1]
    t0 = dt.datetime(2000, 1, 1, *start)
    t1 = dt.datetime(2000, 1, 1, *end)
    diff = t1-t0
    difference = abs(diff.total_seconds())
    return difference

def combine_metrics(sub_list, my_measure, type_raw=True):
# Collects the mean, median, and zscore for each trial.
# Outputs all onto one metric file.
    # # Gather individual data information onto a dictionary.
    # dist_file = 'Indiv Distribution/'+my_measure+'.csv'
    # with open(dist_file, mode='r') as infile:
    #     reader = csv.reader(infile)
    #     next(reader)  # skips header
    #     # {subject: mean, standard deviation}
    #     indiv_dict = {int(rows[0]):[float(rows[1]), float(rows[2])] \
    #                     for rows in reader}
    #
    # dist_file = 'Metric Distribution/'+my_measure+'/ChangeDirPerc.csv'
    # with open(dist_file, mode='r') as infile:
    #     reader = csv.reader(infile)
    #     next(reader)  # skips header
    #     # {subject: mean, standard deviation}
    #     change_dir_dict = {int(rows[0]):[float(rows[1]), float(rows[2])] \
    #                     for rows in reader}
    # dist_file = 'Metric Distribution/'+my_measure+'/NumSpikes.csv'
    # with open(dist_file, mode='r') as infile:
    #     reader = csv.reader(infile)
    #     next(reader)  # skips header
    #     # {subject: mean, standard deviation}
    #     spike_dict = {int(rows[0]):[float(rows[1]), float(rows[2])] \
    #                     for rows in reader}


    # Create metric file with columns names.
    dir = "temp_delete/"
    if not os.path.exists(dir):
        os.makedirs(dir)
    all_metrics = dir+my_measure+'_raw.csv'
    # if type_raw:
    #     columns = ['Subject', 'Control', 'Complexity', 'Mean', 'Zscore']
    #     # columns = ['Subject', 'Control', 'Complexity', 'Mean', 'Median', 'Zscore', \
    #             # 'NumSpikes', 'SpikeZscore', 'ChangeDirPerc', 'ChangeDirZscore', 'IncrTimePerc']
    # else:
    #     columns = ['Subject', 'Control', 'Complexity', 'Mean', 'Zscore']
    #     # columns = ['Subject', 'Control', 'Complexity', 'Mean', 'Median', 'Zscore', \
    #     #         'NumSpikes', 'SpikeZscore', 'ChangeDirPerc', 'ChangeDirZscore']
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
            # std = std_metric(metric_path+file)
            # median = med_metric(metric_path+file)
            # [indiv_mean, indiv_std] = [indiv_dict[sub][0], indiv_dict[sub][1]]
            # zscore = zscore_metric(mean, mean, std)
            zscore = zscore_metric(mean, mean_sub, std_sub)
            # [change_dir_mean, change_dir_std] = [change_dir_dict[sub][0], change_dir_dict[sub][1]]
            # Finding spike data
            # df_spikes = pd.read_csv('Spikes/'+my_measure+'/Sub'+subject+'/'+file)
            # num_spikes = len(df_spikes)
            # [spike_mean, spike_std] = [spike_dict[sub][0], spike_dict[sub][1]]
            # spike_zscore = zscore_metric(num_spikes, spike_mean, spike_std)
            basic_data = [subject, control, complexity, mean, zscore]
            data_list = [*basic_data]
            # basic_data = [subject, control, complexity, mean, median, zscore]
            # if type_raw:
            #     [change_dir_perc, i_count_perc, i_time_perc] = direction_metric(metric_path+file)
            #     change_dir_zcore = zscore_metric(change_dir_perc, change_dir_mean, change_dir_std)
            #     data_list = [*basic_data, num_spikes, spike_zscore, \
            #                 change_dir_perc, change_dir_zcore, i_time_perc]
            # else:
            #     [change_dir_perc, i_count_perc] = direction_notime_metric(metric_path+file)
            #     change_dir_zcore = zscore_metric(change_dir_perc, change_dir_mean, change_dir_std)
            #     data_list = [*basic_data, num_spikes, spike_zscore, \
            #                 change_dir_perc, change_dir_zcore]

            with open(all_metrics, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(data_list)
        bar.next()
    bar.finish()

# def raw_to_metricfile(sub_list, measure, my_measure):
    if type_raw:
        # outputs into Indiv Heart
        if need_original:
            get_raw_heart(sub_list, measure, my_measure)
            print(f"I got the raw {measure} and called it {my_measure}")
        # outputs into Trials
        if need_trials:
            select_trials(sub_list, my_measure)
            print(f"I put the {my_measure} into trials.")
    else:
        # outputs into Indiv Heart
        if need_original:
            get_raw_heart(sub_list, measure, my_measure, measure_type="Analyzed")
            print(f"I got the analyzed {measure} and called it {my_measure}")
        # outputs into Trials
        if need_trials:
            select_trials(sub_list, my_measure, frequency_step=0)
            print(f"I put the {my_measure} into trials.")

    # if need_filtering == True:
    #     # outputs into Trials Filtered
    #     filter_data(sub_list, my_measure, threshold=240)
    #     print(f"I filtered the {my_measure} into Trials Filtered.")
    # if need_indiv_distribution == True:
    #     indiv_distribution(sub_list, my_measure)
    #     print(f"I found the distibution of {my_measure} for each subject.")
    # if need_metricfile == True:
    #     # outputs into Metric Files/my_measure
    #     combine_metrics(sub_list, my_measure)
    #     print(f"We have a metric file for {my_measure}!")


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

need_original = False
need_trials = False
need_filtering = False
need_indiv_distribution = False
plot_each = False
plot_hist_scatter = False
need_metricfile = False
type_raw = False
