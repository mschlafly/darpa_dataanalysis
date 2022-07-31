# Imports
import csv
import matplotlib.pyplot as plt
import numpy as np

def use_sub(required_trials,trial_happened_list, expertise, only_experts, only_novices):
    include_sub = False
    if np.sum(trial_happened_list)>=required_trials:
        if only_experts:
            if expertise == "expert":
                include_sub = True
        elif only_novices:
            if expertise == "novice":
                include_sub = True
        else:
            include_sub = True
    return include_sub

def parse_performance_data(file,subID,environments,control,plot_each):

    lives = np.zeros(10)
    treasure = np.zeros(10)
    input = np.zeros(10)
    difficulty = np.zeros(10)
    score = np.zeros(10)
    trial_happened = np.zeros(10)

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
                            lives[i] = row[3]
                            treasure[i] = row[4]
                            input[i] = row[5]
                            difficulty[i] = row[6]
                            score[i] = lives[i]*3. + treasure[i]
                            trial_happened[i] = 1
                # If the row is for a high complexity trial,
                # store data in the last 5 columns
                else:
                    for i in range(len(control)):
                        if row[1] == control[i]:
                            lives[i+5] = row[3]
                            treasure[i+5] = row[4]
                            input[i+5] = row[5]
                            difficulty[i+5] = row[6]
                            score[i+5] = lives[i+5]*3. + treasure[i+5]
                            trial_happened[i+5] = 1

    # Plot bar graph for individual subject
    if plot_each:
        plt.figure(0)
        width = 0.5
        ind = np.arange(10)
        p1 = plt.bar(ind, lives, width)
        p2 = plt.bar(ind, treasure, width, bottom=lives)
        plt.ylabel('Score')
        plt.title('Game Performance for Subject ' + subID)
        labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                  'HN', 'HW', 'HD', 'HS', 'HA')
        plt.xticks(ind, labels)
        plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
        plt.savefig('Plots/Indiv Plots/' + subID + '_performance.png')

        plt.figure(0+1)
        width = 0.5
        ind = np.arange(10)
        p1 = plt.bar(ind, input, width)
        plt.ylabel('Score')
        plt.title('Inputs for Subject ' + subID)
        labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                  'HN', 'HW', 'HD', 'HS', 'HA')
        plt.xticks(ind, labels)
        plt.savefig('Plots/Indiv Plots/' + subID + '_input.png')

        plt.figure(0+2)
        width = 0.5
        ind = np.arange(10)
        p1 = plt.bar(ind, difficulty, width)
        plt.ylabel('Score')
        plt.title('Difficulty for Subject ' + subID)
        labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                  'HN', 'HW', 'HD', 'HS', 'HA')
        plt.xticks(ind, labels)
        plt.savefig('Plots/Indiv Plots/' + subID + '_difficulty.png')
        plt.close('all')

    return [lives, treasure, input, difficulty, score, trial_happened]

def parse_cogload_data(file,subID,environments,control,plot_each):

    subID_RR = 'Sub'+subID
    RR_mean = np.zeros(10)
    RR_zscore = np.zeros(10)
    trial_happened = np.zeros(10)

    with open(file, 'r') as csvfile:
        data = csv.reader(csvfile, delimiter=',')

        # Loop through rows in csv with each row representing a trial.
        # If the row is for the subject of interest, act
        for row in data:
            # print(row)
            if row[0] == subID_RR:
                # If the row is for a low complexity trial,
                # store data in the first 5 columns
                if row[2] == environments[0]:
                    for i in range(len(control)):
                        if row[1] == control[i]:
                            if float(row[8]) > 0:
                                RR_zscore[i] = row[9]#(1/float(row[8]))*1000*60
                                RR_mean[i] = row[8]
                                trial_happened[i] = 1
                # If the row is for a high complexity trial,
                # store data in the last 5 columns
                else:
                    for i in range(len(control)):
                        if row[1] == control[i]:
                            if float(row[8]) > 0:
                                RR_zscore[i+5] = row[9]#(1/float(row[8]))*1000*60
                                RR_mean[i+5] = row[8]
                                trial_happened[i+5] = 1

    # Plot bar graph for individual subject
    if plot_each:
        plt.figure(0)
        width = 0.5
        ind = np.arange(10)
        p1 = plt.bar(ind, RR_mean, width)
        plt.ylabel('Mean RR Interval')
        plt.title('RR Interval for Subject ' + subID)
        labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                  'HN', 'HW', 'HD', 'HS', 'HA')
        plt.xticks(ind, labels)
        plt.savefig('Plots/Indiv Plots/' + subID + '_RR.png')
        plt.close('all')

    return [RR_mean, RR_zscore, trial_happened]

def parse_MDP_data(file,sub,environments,control,plot_each):

    regret_cum = np.zeros(10)
    trial_happened = np.zeros(10)

    # Get string version of subject number for labeling
    if sub < 10:
        subID = '0' + str(sub)
    else:
        subID = str(sub)

    with open(file, 'r') as csvfile:
        data = csv.reader(csvfile, delimiter=',')

        # Loop through rows in csv with each row representing a trial.
        # If the row is for the subject of interest, act
        for row in data:
            if row[0] == str(subID):
                # If the row is for a low complexity trial,
                # store data in the first 5 columns
                if row[2] == environments[0]:
                    for i in range(len(control)):
                        if row[1] == control[i]:
                            regret_cum[i] = row[3]
                            trial_happened[i] = 1
                # If the row is for a high complexity trial,
                # store data in the last 5 columns
                else:
                    for i in range(len(control)):
                        if row[1] == control[i]:
                            regret_cum[i+5] = row[3]
                            trial_happened[i+5] = 1

    # Plot bar graph for individual subject
    if plot_each:
        plt.figure(0)
        width = 0.5
        ind = np.arange(10)
        p1 = plt.bar(ind, regret_cum, width)
        plt.ylabel('Cummulative Regret')
        plt.title('Cummulative Regret for Subject ' + subID)
        labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                  'HN', 'HW', 'HD', 'HS', 'HA')
        plt.xticks(ind, labels)
        plt.savefig('Plots/Indiv Plots/' + subID + '_regret.png')
        plt.close('all')

    return [regret_cum, trial_happened]
