# This program plots overall performance metrics for DAPRA HSR. It also optionally
# saves the data for statistical testing in R. It relies upon functions from make_boxplot.py
# for plotting data and statistical results entered by hand. It can optionally look at
# only experts or novices or plot various other comparisons.

# Imports
import csv
import numpy as np
import matplotlib.pyplot as plt
from make_boxplot import make_boxplot, add_stats

# Booleans to indicate functions of script.
save_data = True # Saves control comparison data into csv files for statistical processing
save_method_average = False # Saves method comparison data into csv files for statistical processing
plot_each = False # Creates and saves a plot for each participant

# Booleans for analyzing subsets of the data
only_experts = False # Only plots experts
only_novices = False # Only plots novices
plot_method_average = True # metrics for the ergodic control trials are
                    # averaged, then plotted in comparison to other methods

# Indicate range of subjects to include in plots within [1,42]
minsub = 1
maxsub = 42
skipped_subjects = [] # Specify specific subjects to skip here.
    # [2,3,4,5,6,10,12,16,19,15,38] are the subjects with incomplete data - (note, might not be all of them)

# Set filenames for getting raw data
file = "performance.csv"
file_subdata = "subject_info.csv" # contains subject info from the questionaire
        # like the number of hours spent playing video games over their lifetime

# String for experimental conditions
environments = ['low','high']
control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
method = ['none','waypoint','ergodic'] # where ergodic is averaged among the ergodic trials
autonomy = ['direct','shared','auto']

# Set up csvs for storing data to process in R
if save_data:
    # type of control
    file_control_all = "control.csv"
    columns = ['Subject','Control','Complexity','Trial','Skill','Perweek',
                'Lifetime','Expertise','Uselow','Usehigh','Lives','Treasure','Score']
                # 'Uselow' and 'Usehigh' are booleans stating whether you have all of
                # the data for all of the low complexity trials or all of the low complexity trials
    with open(file_control_all,'w') as csvfile:
        testwriter = csv.writer(csvfile,delimiter=',')
        testwriter.writerow(columns)
    # method
    file_method_all = "method_average.csv"
    columns = ['Subject','Method','Complexity','Skill','Perweek',
                'Lifetime','Expertise','Uselow','Usehigh','Lives','Treasure','Score']
    with open(file_method_all,'w') as csvfile:
        testwriter = csv.writer(csvfile,delimiter=',')
        testwriter.writerow(columns)

# Initialize matrices to store participant data. Matrices have rows for every
# participant even though not all have complete data. A variable sub is
# indicated the number of actual subjects with full datasets stored. If a subject's
# dataset is not full, their row is overwritten. These matrices are in the format for boxplots
subnum = 0
# These matrices are filled while looping through raw data
maxsub +=1
lives_all = np.zeros((maxsub,10))
treasure_all = np.zeros((maxsub,10))
score_all = np.zeros((maxsub,10))
lives_all_list = [0] * 10
treasure_all_list = [0] * 10
score_all_list = [0] * 10
# These matrices are filled at the end of the loop
if plot_method_average:
    lives_method = np.zeros((maxsub,6))
    treasure_method = np.zeros((maxsub,6))
    score_method = np.zeros((maxsub,6))
    lives_method_list = [0] * 6
    treasure_method_list = [0] * 6
    score_method_list = [0] * 6

# Look through all participants
for sub in range(minsub, maxsub):
    trial_happened = np.zeros(10)
    # Skips subjects numbers in the skipped_subjects list
    found = False
    for i in range(len(skipped_subjects)):
        if sub==skipped_subjects[i]:
            found = True

    if found == False:
        # Get string version of subject number for labeling
        if sub<10:
            subID = '0' + str(sub)
        else:
            subID = str(sub)

        # Import data for parsing
        with open(file,'r') as csvfile:
            data = csv.reader(csvfile,delimiter=',')

            # Loop through rows in csv with each row representing a trial.
            # If the row is for the subject of interest, act
            for row in data:
                if row[0]==subID:
                    # If the row is for a low complexity trial, store data in the first 5 columns
                    if row[2]==environments[0]:
                        for i in range(len(control)):
                            if row[1]==control[i]:
                                lives_all[sub,i] = row[3]
                                treasure_all[sub,i] = row[4]
                                score_all[sub,i] = lives_all[sub,i]*3. + treasure_all[sub,i]
                                trial_happened[i] = 1
                    # If the row is for a high complexity trial, store data in the last 5 columns
                    else:
                        for i in range(len(control)):
                            if row[1]==control[i]:
                                lives_all[sub,i+5] = row[3]
                                treasure_all[sub,i+5] = row[4]
                                score_all[sub,i+5] = lives_all[sub,i+5]*3. + treasure_all[sub,i+5]
                                trial_happened[i+5] = 1

            # Plot bar graph for individual subject
            if plot_each:
                plt.figure(sub)
                width = 0.5
                ind = np.arange(10)
                p1 = plt.bar(ind,lives_all[sub,:],width)
                p2 = plt.bar(ind,treasure_all[sub,:],width,bottom=lives_all[sub,:])
                plt.ylabel('Score')
                plt.title('Game Performance for Subject ' + subID)
                labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
                plt.xticks(ind, labels)
                plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
                plt.savefig('individualplots/'+subID+'_performance.png')

            # Read subject_info.csv for the amount of video games the person plays
            with open(file_subdata,'r') as csvfile:
                subdata = csv.reader(csvfile)#,delimiter=',')
                for row in subdata:
                    if str(sub)==row[0]:
                        skill = np.mean(lives_all[sub,:])
                        perweek = row[1]
                        lifetime = row[2]
                        if float(lifetime+'.0')>999.0:
                            expertise = "expert"
                        else:
                            expertise = "novice"

            # Check to see if we have data for all low complexity trials and all
            # high complexity trials seperately. Makes the assumption that no
            # trial ended with 0 lives
            if np.min(lives_all[sub,0:5])>0:
                all_low = 1
            else:
                all_low = 0

            if np.min(lives_all[sub,5:10])>0:
                all_high = 1
            else:
                all_high = 0

            # if all of the experimental trials are there for either all high or all low environmental complexity
            if (all_high==1 or all_low==1):
                # Saves data for statistical tests in R
                if save_data:
                    with open(file_control_all,'a') as csvfile:
                        testwriter = csv.writer(csvfile,delimiter=',')
                        for i in range(10):
                            if i<5:
                                con = i
                                env = 0
                            else:
                                con = i-5
                                env = 1
                            row_save = ["Sub"+subID,control[con],environments[env],'trial'+str(i),
                                        skill,perweek,lifetime,expertise,all_low,all_high,
                                        lives_all[sub,i],treasure_all[sub,i],score_all[sub,i]]
                            testwriter.writerow(row_save)

                # Saves average ergodic trial data in array for plotting
                if plot_method_average:
                    # Loop through none and waypoint files
                    for i in range(10):
                        if i<5:
                            con = i
                            env = 0
                            ind = i
                        else:
                            con = i-5
                            env = 1
                            ind = i-2
                        if control[con] == 'none' or control[con] == 'waypoint':
                            lives_method[sub,ind] = lives_all[sub,i]
                            treasure_method[sub,ind] = treasure_all[sub,i]
                            score_method[sub,ind] = score_all[sub,i]

                    # Save average ergodic low performance
                    con = 2
                    env = 0
                    ind = 2
                    lives_average = np.mean(lives_all[sub,2:5])
                    treasure_average = np.mean(treasure_all[sub,2:5])
                    score_average = np.mean(score_all[sub,2:5])
                    lives_method[sub,ind] = lives_average
                    treasure_method[sub,ind] = treasure_average
                    score_method[sub,ind] = score_average

                    # Save average ergodic high performance
                    con = 2
                    env = 1
                    ind = 5
                    lives_average = np.mean(lives_all[sub,7:10])
                    treasure_average = np.mean(treasure_all[sub,7:10])
                    score_average = np.mean(score_all[sub,7:10])
                    lives_method[sub,ind] = lives_average
                    treasure_method[sub,ind] = treasure_average
                    score_method[sub,ind] = score_average

                # Saves average ergodic trial data in csv for stat tests
                if save_method_average:
                    with open(file_method_all,'a') as csvfile:
                        testwriter = csv.writer(csvfile,delimiter=',')

                        # Loop through none and waypoint files
                        for i in range(10):
                            if i<5:
                                con = i
                                env = 0
                                ind = i
                            else:
                                con = i-5
                                env = 1
                                ind = i-2
                            if control[con] == 'none' or control[con] == 'waypoint':
                                row_save = ["Sub"+subID,method[con],environments[env],
                                            skill,perweek,lifetime,expertise,all_low,all_high,
                                            lives_all[sub,i],treasure_all[sub,i],score_all[sub,i]]
                                testwriter.writerow(row_save)
                        # Save average ergodic low performance
                        con = 2
                        env = 0
                        ind = 2
                        lives_average = np.mean(lives_all[sub,2:5])
                        treasure_average = np.mean(treasure_all[sub,2:5])
                        score_average = np.mean(score_all[sub,2:5])
                        row_save = ["Sub"+subID,method[con],environments[env],
                                    skill,perweek,lifetime,expertise,all_low,all_high,
                                    lives_average,treasure_average,score_average]
                        testwriter.writerow(row_save)

                        # Save average ergodic high performance
                        con = 2
                        env = 1
                        ind = 5
                        lives_average = np.mean(lives_all[sub,7:10])
                        treasure_average = np.mean(treasure_all[sub,7:10])
                        score_average = np.mean(score_all[sub,7:10])
                        row_save = ["Sub"+subID,method[con],environments[env],
                                    skill,perweek,lifetime,expertise,all_low,all_high,
                                    lives_average,treasure_average,score_average]
                        testwriter.writerow(row_save)

            # Decide whether the subject should be included, default is to skip and
            # that is overwritten if we have lives data for every trial.
            # If we are only looking at either experts or novies, the participant
            # has to additionally belong to that group
            include_sub = False
            if (all_low==1) or (all_high==1):
                if only_experts:
                    if expertise=="expert":
                        include_sub = True
                elif only_novices:
                    if expertise=="novice":
                        include_sub = True
                else:
                    include_sub = True

            if include_sub:
                subnum += 1
                for i in range(10):
                    if trial_happened[i] == 1:
                        if isinstance(lives_all_list[i],int):
                            lives_all_list[i] = lives_all[sub,i]
                            treasure_all_list[i] = treasure_all[sub,i]
                            score_all_list[i] = score_all[sub,i]
                        else:
                            lives_all_list[i] = np.append(lives_all_list[i],lives_all[sub,i])
                            treasure_all_list[i] = np.append(treasure_all_list[i],treasure_all[sub,i])
                            score_all_list[i] = np.append(score_all_list[i],score_all[sub,i])
                if plot_method_average:
                    if all_low==1:
                        for i in range(3):
                            if isinstance(lives_method_list[i],int):
                                lives_method_list[i] = lives_method[sub,i]
                                treasure_method_list[i] = treasure_method[sub,i]
                                score_method_list[i] = score_method[sub,i]
                            else:
                                lives_method_list[i] = np.append(lives_method_list[i],lives_method[sub,i])
                                treasure_method_list[i] = np.append(treasure_method_list[i],treasure_method[sub,i])
                                score_method_list[i] = np.append(score_method_list[i],score_method[sub,i])
                    if all_high==1:
                        for i in range(3,6):
                            if isinstance(lives_method_list[i],int):
                                lives_method_list[i] = lives_method[sub,i]
                                treasure_method_list[i] = treasure_method[sub,i]
                                score_method_list[i] = score_method[sub,i]
                            else:
                                lives_method_list[i] = np.append(lives_method_list[i],lives_method[sub,i])
                                treasure_method_list[i] = np.append(treasure_method_list[i],treasure_method[sub,i])
                                score_method_list[i] = np.append(score_method_list[i],score_method[sub,i])
print('The number of subjects included in boxplots is ', subnum)

# PLOT BOXPLOTS

############################################
# Overall Plots
############################################
# Plot parameters
labels = ('L-None','L-WP','L-Direct','L-Shared','L-Auto','H-None','H-WP','H-Direct','H-Shared','H-Auto')
box_colors = ['#b71c1c','#ff6f00','#1b5e20','#006064','#1a237e','#b71c1c','#ff6f00','#1b5e20','#006064','#1a237e']
box_alpha = [0.5,0.5,0.5,0.5,0.5,None,None,None,None,None] # sets how transparent to make the color
figure_size = (6,3.55) # sets the size of the figure in inches
xlabel = 'Experimental Condition'

data = lives_all_list
ylabel = 'Number of lives at the end of the game'
if only_experts:
    title = 'Lives Leftover for Experts'
    sig_matrix = np.array([])
elif only_novices:
    title = 'Lives Leftover for Novices'
    sig_matrix = np.array([])
else:
    title = 'Lives Leftover'
    sig_matrix = np.array([])
[fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
add_stats(data,sig_matrix,ax)
if only_experts:
    fig.savefig('aggregateplots/'+'overall_lives_experts.png')
elif only_novices:
    fig.savefig('aggregateplots/'+'overall_lives_novices.png')
else:
    fig.savefig('aggregateplots/'+'overall_lives.png')

data = treasure_all_list
ylabel = 'Number of targets collected'
if only_experts:
    title = 'Targets Collected for Experts'
    sig_matrix = np.array([])
elif only_novices:
    title = 'Targets Collected for Novices'
    sig_matrix = np.array([])
else:
    title = 'Targets Collected'
    sig_matrix = np.array([])
[fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
add_stats(data,sig_matrix,ax)
if only_experts:
    fig.savefig('aggregateplots/'+'overall_treas_experts.png')
elif only_novices:
    fig.savefig('aggregateplots/'+'overall_treas_novices.png')
else:
    fig.savefig('aggregateplots/'+'overall_treas.png')

data = score_all_list
ylabel = 'Final score = #lives*3+#treasure'
if only_experts:
    title = 'Final Score for Experts'
    sig_matrix = np.array([
                            # [7,8,.0653/2] # high autoergodic - low autoergodic
                            # [7,8,.0026/2] # low autoergodic - high sharedergodic
                            [4,3,.0089/2], # low autoergodic - low sharedergodic
                            # [7,8,.0352/2] # high sharedergodic - low waypoint
                            [3,1,.0956/2] # low sharedergodic - low waypoint
                            ])
    # sig_matrix = np.array([])
elif only_novices:
    title = 'Final Score for Novices'
    # sig_matrix = np.array([
    #                         [9,7,.000001/2], # high autoergodic - high directergodic
    #                         [9,5,.0844/2], # high autoergodic - high none
    #                         # [9,3,.0238/2], # high autoergodic - low sharedergodic
    #                         [9,6,.0178/2], # high autoergodic - high waypoint
    #                         # [4,7,.0679/2], # low autoergodic - high directergodic
    #                         # [7,2,.0632/2], # high directergodic - low directergodic
    #                         [7,8,.0294/2], # high directergodic - high sharedergodic
    #                         # [7,6,.0092/2], # high directergodic - low waypoint
    #                         ])
    sig_matrix = np.array([])
else:
    title = 'Final Score'
    sig_matrix = np.array([])
[fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
add_stats(data,sig_matrix,ax)
if only_experts:
    fig.savefig('aggregateplots/'+'overall_score_experts.png')
elif only_novices:
    fig.savefig('aggregateplots/'+'overall_score_novices.png')
else:
    fig.savefig('aggregateplots/'+'overall_score.png')

############################################
# Method Plots
############################################
if plot_method_average:
    # Plot parameters
    labels = ('L-None','L-WP','L-Ergodic','H-None','H-WP','H-Ergodic')
    box_colors = ['#b71c1c','#ff6f00','#006064','#b71c1c','#ff6f00','#006064']
    box_alpha = [0.5,0.5,0.5,None,None,None]
    figure_size = (6,3.55) # sets the size of the figure in inches
    xlabel = 'Experimental Condition'

    print(lives_method_list)
    data = lives_method_list
    # title = 'Lives Leftover for Different Control Methods'
    title = 'Lives Leftover for None, Waypoint, and Direct Ergodic'
    ylabel = 'Number of lives at the end of the game'
    [fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
    fig.savefig('aggregateplots/'+'method_lives.png')

    data = treasure_method_list
    # title = 'Treasures Found for Different Control Methods'
    title = 'Treasures Found for None, Waypoint, and Direct Ergodic'
    ylabel = 'Number of treasures found'
    [fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
    fig.savefig('aggregateplots/'+'method_treas.png')

    data = score_method_list
    # title = 'Final Score for Different Control Methods'
    title = 'Final Score for None, Waypoint, and Direct Ergodic'
    ylabel = 'Final score = #lives*3+#treasure'
    [fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
    fig.savefig('aggregateplots/'+'method_score.png')
