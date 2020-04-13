import csv
import numpy as np
import matplotlib.pyplot as plt
from make_boxplot import make_boxplot, add_stats


minsub = 1
maxsub = 42
skipped_subjects = []#[2,3,4,5,6,10,12,16,19,15,38]

plot_each = False
save_data = True
only_experts = False
only_novices = False

file = "performance.csv"
file_subdata = "subject_info.csv"
environments = ['low','high']
control = ['none','waypoint','directergodic','sharedergodic','autoergodic']
method = ['none','waypoint','ergodic']
autonomy = ['direct','shared','auto']

# # Count the number of subjects tested
# numsub = 0
# for sub in range(minsub, maxsub+1):
#     found = False
#     for i in range(len(skipped_subjects)):
#         # print(list_of_complete_datasets[i])
#         if sub==skipped_subjects[i]:
#             found = True
#     if found == False:
#         numsub +=1

# Set up csvs for storing data to process in R
if save_data:
    # type of control
    file_control_all = "control.csv"
    columns = ['Subject','Control','Complexity','Trial','Skill','Perweek',
                'Lifetime','Expertise','Uselow','Usehigh','Lives','Treasure','Score']
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
    # autonomy level
    file_autonomy_all = "autonomy_all.csv"
    columns = ['Subject','Autonomy','Complexity','Skill','Perweek',
                'Lifetime','Expertise','Uselow','Usehigh','Lives','Treasure','Score']
    with open(file_autonomy_all,'w') as csvfile:
        testwriter = csv.writer(csvfile,delimiter=',')
        testwriter.writerow(columns)

lives_all = np.zeros((maxsub,10))
treasure_all = np.zeros((maxsub,10))
score_all = np.zeros((maxsub,10))
lives_method = np.zeros((maxsub,6))
treasure_method = np.zeros((maxsub,6))
score_method = np.zeros((maxsub,6))
lives_autonomy = np.zeros((maxsub,6))
treasure_autonomy = np.zeros((maxsub,6))
score_autonomy = np.zeros((maxsub,6))
subnum = 0
for sub in range(minsub, maxsub+1):
    found = False
    for i in range(len(skipped_subjects)):
        # print(list_of_complete_datasets[i])
        if sub==skipped_subjects[i]:
            found = True
    if found == False:

        if sub<10:
            subID = '0' + str(sub)
        else:
            subID = str(sub)
        # Import data for parsing
        with open(file,'r') as csvfile:
            data = csv.reader(csvfile,delimiter=',')
            # print(lives_all[subnum,:])
            for row in data:
                # print(row)
                if row[0]==str(sub):
                    if row[2]==environments[0]:
                        for i in range(len(control)):
                            if row[1]==control[i]:
                                lives_all[subnum,i] = row[3]
                                treasure_all[subnum,i] = row[4]
                                score_all[subnum,i] = lives_all[subnum,i]*3. + treasure_all[subnum,i]
                                # print(lives_all[subnum,:])
                    else:
                        for i in range(len(control)):
                            if row[1]==control[i]:
                                lives_all[subnum,i+5] = row[3]
                                treasure_all[subnum,i+5] = row[4]
                                score_all[subnum,i+5] = lives_all[subnum,i+5]*3. + treasure_all[subnum,i+5]
            # print(lives_all[subnum,:])
            # Plot bar graph for individual subject
            if plot_each:
                plt.figure(sub)
                width = 0.5
                ind = np.arange(10)
                p1 = plt.bar(ind,lives_all[subnum,:],width)
                p2 = plt.bar(ind,lives_all[subnum,:],width,bottom=lives_all[subnum,:])
                plt.ylabel('Score')
                plt.title('Game Performance for Subject ' + subID)
                labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
                plt.xticks(ind, labels)
                plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
                plt.savefig('individualplots/'+subID+'_performance.pdf')

            if save_data:
                # Read subject_info.csv for the amount of video games the person plays
                with open(file_subdata,'r') as csvfile:
                    subdata = csv.reader(csvfile)#,delimiter=',')
                    for row in subdata:
                        if str(sub)==row[0]:
                            skill = np.mean(lives_all[subnum,:])
                            perweek = row[1]
                            lifetime = row[2]
                            if float(lifetime+'.0')>999.0:
                                expertise = "expert"
                            else:
                                expertise = "novice"

                if np.min(lives_all[subnum,0:5])>0:
                    all_low = 1
                else:
                    all_low = 0

                if np.min(lives_all[subnum,5:10])>0:
                    all_high = 1
                else:
                    all_high =0

                # if all of the experimental trials are there for either all high or all low
                if (all_high==1 or all_low==1):
                    # control type
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
                                        lives_all[subnum,i],treasure_all[subnum,i],score_all[subnum,i]]
                            testwriter.writerow(row_save)
                    # method
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
                                            lives_all[subnum,i],treasure_all[subnum,i],score_all[subnum,i]]
                                testwriter.writerow(row_save)
                                lives_method[subnum,ind] = lives_all[subnum,i]
                                treasure_method[subnum,ind] = treasure_all[subnum,i]
                                score_method[subnum,ind] = score_all[subnum,i]

                        # Save average ergodic low performance
                        ind_control = 4

                        con = 2
                        env = 0
                        ind = 2
                        lives_average = np.mean(lives_all[subnum,2:5])
                        treasure_average = np.mean(treasure_all[subnum,2:5])
                        score_average = np.mean(score_all[subnum,2:5])
                        row_save = ["Sub"+subID,method[con],environments[env],
                                    skill,perweek,lifetime,expertise,all_low,all_high,
                                    lives_average,treasure_average,score_average]
                        testwriter.writerow(row_save)
                        lives_method[subnum,ind] = lives_average
                        treasure_method[subnum,ind] = treasure_average
                        score_method[subnum,ind] = score_average

                        # Save average ergodic high performance
                        con = 2
                        env = 1
                        ind = 5
                        lives_average = np.mean(lives_all[subnum,7:10])
                        treasure_average = np.mean(treasure_all[subnum,7:10])
                        score_average = np.mean(score_all[subnum,7:10])
                        row_save = ["Sub"+subID,method[con],environments[env],
                                    skill,perweek,lifetime,expertise,all_low,all_high,
                                    lives_average,treasure_average,score_average]
                        testwriter.writerow(row_save)
                        lives_method[subnum,ind] = lives_average
                        treasure_method[subnum,ind] = treasure_average
                        score_method[subnum,ind] = score_average
                    # autonomy
                    with open(file_autonomy_all,'a') as csvfile:
                        testwriter = csv.writer(csvfile,delimiter=',')
                        for i in range(10):
                            if i<5:
                                con = i
                                env = 0
                                ind = i-2
                            else:
                                con = i-5
                                env = 1
                                ind = i-4
                            if con>1:
                                row_save = ["Sub"+subID,autonomy[con-2],environments[env],
                                            skill,perweek,lifetime,expertise,all_low,all_high,
                                            lives_all[subnum,i],treasure_all[subnum,i],score_all[subnum,i]]
                                testwriter.writerow(row_save)
                                lives_autonomy[subnum,ind] = lives_all[subnum,i]
                                treasure_autonomy[subnum,ind] = treasure_all[subnum,i]
                                score_autonomy[subnum,ind] = score_all[subnum,i]

            # print(sub,np.min(lives_all[subnum,:]),lives_all[subnum,:])
            skip_sub = True
            if np.min(lives_all[subnum,:])>0:# and expertise=="expert": #skill>6.69: #skill>6.91:# and np.mean(lives_all[subnum,:])>7:
                # print(subnum,sub,lifetime)#,lives_all[subnum,:])
                sub_prev = subnum
                if only_experts:
                    if expertise=="expert":
                        subnum += 1
                        skip_sub = False
                elif only_novices:
                    if expertise=="novice":
                        # print(subID,lives_all[subnum,:])
                        subnum += 1
                        skip_sub = False
                else:
                    subnum += 1
                    skip_sub = False

            if skip_sub:
                lives_all[subnum,:] = np.zeros(10)
                treasure_all[subnum,:] = np.zeros(10)
                score_all[subnum,:] = np.zeros(10)
                lives_method[subnum,:] = np.zeros(6)
                treasure_method[subnum,:] = np.zeros(6)
                score_method[subnum,:] = np.zeros(6)
                lives_autonomy[subnum,:] = np.zeros(6)
                treasure_autonomy[subnum,:] = np.zeros(6)
                score_autonomy[subnum,:] = np.zeros(6)


print('The number of subjects included is ', subnum)

# PLOT BOXPLOTS
############################################
# Overall Plots
############################################
# Plot parameters
labels = ('L-None','L-WP','L-Direct','L-Shared','L-Auto','H-None','H-WP','H-Direct','H-Shared','H-Auto')
box_colors = ['#b71c1c','#ff6f00','#1b5e20','#006064','#1a237e','#b71c1c','#ff6f00','#1b5e20','#006064','#1a237e']
box_alpha = [0.5,0.5,0.5,0.5,0.5,None,None,None,None,None]
figure_size = (6,3.55) # sets the size of the figure in inches
xlabel = 'Experimental Condition'

data = lives_all[:subnum,:]
ylabel = 'Number of lives at the end of the game'
if only_experts:
    title = 'Lives Leftover for Experts'
    sig_matrix = np.array([[1,4,0.0533/2],
                            [3,4,0.0156/2],
                            [5,8,0.0693/2],
                            [4,9,.092/2]])
elif only_novices:
    title = 'Lives Leftover for Novices'
    sig_matrix = np.array([])
else:
    title = 'Lives Leftover'
    sig_matrix = np.array([[1,4,.0255],
                            [3,4,.0454],
                            [6,9,.0335],
                            [4,9,.0041]])
[fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
add_stats(data,sig_matrix,ax)
if only_experts:
    fig.savefig('overall_experts_lives.pdf')
elif only_novices:
    fig.savefig('overall_novices_lives.pdf')
else:
    fig.savefig('overall_lives.pdf')


data = treasure_all[:subnum,:]
ylabel = 'Number of targets collected'
if only_experts:
    title = 'Targets Collected for Experts'
    # sig_matrix = np.array([[1,3,0.00206],
    #                         [3,4,0.0109],
    #                         [3,8,0.025]])
    sig_matrix = np.array([[0,1,.08296/2],
                            [0,3,.0881/2],
                            [1,3,.00206/2],
                            [2,3,.0695/2],
                            [3,4,.0109/2],
                            [5,6,.08296/2],
                            [3,8,.024998/2]])
elif only_novices:
    title = 'Targets Collected for Novices'
    sig_matrix = np.array([[2,3,.015],
                            [5,9,.015],
                            [6,9,.000482]])
else:
    title = 'Targets Collected'
    sig_matrix = np.array([[0,1,.0432],
                            [0,3,.0195],
                            [1,3,.000359],
                            [2,3,.00459],
                            [3,4,.00797],
                            [5,6,.0231],
                            [3,8,.00975]])
[fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
add_stats(data,sig_matrix,ax)
if only_experts:
    fig.savefig('overall_experts_treas.pdf')
elif only_novices:
    fig.savefig('overall_novices_treas.pdf')
else:
    fig.savefig('overall_treas.pdf')

data = score_all[:subnum,:]
ylabel = 'Final score = #lives*3+#treasure'
if only_experts:
    title = 'Final Score for Experts'
    sig_matrix = np.array([[3,4,0.00437],
                            [4,9,0.0294]])
    sig_matrix = np.array([[2,3,.0818/2],
                            [3,4,.00437/2],
                            [4,9,.0294/2]])
elif only_novices:
    title = 'Final Score For novices'
    sig_matrix = np.array([[6,9,.0266],
                            [7,8,.0473],
                            [8,9,.0456]])
else:
    title = 'Final Score'
    sig_matrix = np.array([[3,4,.0189],
                            [6,9,.0194],
                            [4,9,.00342]])
[fig,ax]=make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
add_stats(data,sig_matrix,ax)
if only_experts:
    fig.savefig('overall_experts_score.pdf')
elif only_novices:
    fig.savefig('overall_novices_score.pdf')
else:
    fig.savefig('overall_score.pdf')

############################################
# Method Plots
############################################
# Plot parameters
labels = ('L-None','L-WP','L-Ergodic','H-None','H-WP','H-Ergodic')
box_colors = ['#b71c1c','#ff6f00','#006064','#b71c1c','#ff6f00','#006064']
box_alpha = [0.5,0.5,0.5,None,None,None]
figure_size = (6,3.55) # sets the size of the figure in inches
xlabel = 'Experimental Condition'

data = lives_method[:subnum,:]
# title = 'Lives Leftover for Different Control Methods'
title = 'Lives Leftover for None, Waypoint, and Direct Ergodic'
ylabel = 'Number of lives at the end of the game'
make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
plt.savefig('method_lives.pdf')

data = treasure_method[:subnum,:]
# title = 'Treasures Found for Different Control Methods'
title = 'Treasures Found for None, Waypoint, and Direct Ergodic'
ylabel = 'Number of treasures found'
make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
plt.savefig('method_treas.pdf')

data = score_method[:subnum,:]
# title = 'Final Score for Different Control Methods'
title = 'Final Score for None, Waypoint, and Direct Ergodic'
ylabel = 'Final score = #lives*3+#treasure'
make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
plt.savefig('method_score.pdf')

############################################
# Autonomy Plots
############################################
# Plot parameters
labels = ('L-Direct','L-Shared','L-Auto','H-Direct','H-Shared','H-Auto')
box_colors = ['#1b5e20','#006064','#1a237e','#1b5e20','#006064','#1a237e']
box_alpha = [0.5,0.5,0.5,None,None,None]
figure_size = (6,3.55) # sets the size of the figure in inches
xlabel = 'Experimental Condition'

data = lives_autonomy[:subnum,:]
title = 'Lives Leftover for the Different Levels of Autonomy'
ylabel = 'Number of lives at the end of the game'
make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
plt.savefig('autonomy_lives.pdf')

data = treasure_autonomy[:subnum,:]
title = 'Treasures Found for the Different Levels of Autonomy'
ylabel = 'Number of treasures found'
make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
plt.savefig('autonomy_treas.pdf')

data = score_autonomy[:subnum,:]
title = 'Final Score for the Different Levels of Autonomy'
ylabel = 'Final score = #lives*3+#treasure'
make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size)
plt.savefig('autonomy_score.pdf')

autonomy_count = np.zeros(3)
method_count = np.zeros(4)
with open(file_subdata,'r') as csvfile:
    subdata = csv.reader(csvfile)#,delimiter=',')
    for row in subdata:
        # print(row[3])
        if row[3]=="direct'":
            autonomy_count[0] +=1
        elif row[3]=="shared'":
            autonomy_count[1] +=1
        elif row[3]=="auto'":
            autonomy_count[2] +=1

        if row[4]=="none'":
            method_count[0] +=1
        elif row[4]=="path'":
            method_count[1] +=1
        elif row[4]=="shade'":
            method_count[2] +=1
        elif row[4]=="no control'":
            method_count[3] +=1
    print('The number of participants who gave method feedback is '+str(np.sum(method_count)))
    print('The number of participants who gave autonomy feedback is '+str(np.sum(autonomy_count)))


plt.figure(50,dpi=150)
width = 0.5
ind = np.arange(4)
print(method_count)
p1 = plt.bar(ind,method_count,width)
plt.ylabel('Number of Participants')
plt.title('Participant feedback for preferred method of commanding swarm')
labels = ('no control','pathways','shading','no drones')
plt.xticks(ind, labels, rotation=25)
plt.savefig('method_feedback.pdf')

plt.figure(51,dpi=150)
width = 0.5
ind = np.arange(3)
print(autonomy_count)
p1 = plt.bar(ind,autonomy_count,width)
plt.ylabel('Number of Participants')
plt.title('Participant feedback for preferred level of autonomy')
labels = ('direct','shared','fully autonomous')
plt.xticks(ind, labels, rotation=15)
plt.savefig('autonomy_feedback.pdf')
