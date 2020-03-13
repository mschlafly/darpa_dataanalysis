import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon



minsub = 1
maxsub = 42
skipped_subjects = [2,3,4,5,6,10,12,16,19,15,38]

plot_each = True

file = "performance.csv"
environments = ['low','high']
control = ['none','waypoint','directergodic','sharedergodic','autoergodic']

# Count the number of subjects tested
numsub = 0
for sub in range(minsub, maxsub+1):
    found = False
    for i in range(len(skipped_subjects)):
        # print(list_of_complete_datasets[i])
        if sub==skipped_subjects[i]:
            found = True
    if found == False:
        numsub +=1
print('Plotting data for '+str(numsub)+' subjects')

# Set up csv for storing data to process in R
file_all = "metrics_all.csv"
columns = ['Subject','Control','Complexity','Trial','Lives','Treasure','Score']
with open(file_all,'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter=',')
    testwriter.writerow(columns)
file_high = "metrics_high.csv"
columns = ['Subject','Control','Lives','Treasure','Score']
with open(file_high,'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter=',')
    testwriter.writerow(columns)
file_low = "metrics_low.csv"
with open(file_low,'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter=',')
    testwriter.writerow(columns)

lives_all = np.zeros((numsub,10))
treasure_all = np.zeros((numsub,10))
score_all = np.zeros((numsub,10))
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
            for row in data:
                if row[0]==subID:
                    if row[2]==environments[0]:
                        for i in range(len(control)):
                            if row[1]==control[i]:
                                lives_all[subnum,i] = row[3]
                                treasure_all[subnum,i] = row[4]
                                score_all[subnum,i] = lives_all[subnum,i]*3. + treasure_all[subnum,i]
                    else:
                        for i in range(len(control)):
                            if row[1]==control[i]:
                                lives_all[subnum,i+5] = row[3]
                                treasure_all[subnum,i+5] = row[4]
                                score_all[subnum,i+5] = lives_all[subnum,i+5]*3. + treasure_all[subnum,i+5]

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
            # if np.min(lives_all[subnum,:])>0:
            with open(file_all,'w') as csvfile:
                testwriter = csv.writer(csvfile,delimiter=',')
                for i in range(10):
                    if i<5:
                        con = i
                        env = 0
                    else:
                        con = i-5
                        env = 1
                    row_save = [subID,control[con],environments[env],'trial'+str(i),
                                lives_all[subnum,i],treasure_all[subnum,i],score_all[subnum,i]]
                    testwriter.writerow(row_save)
            # else:
            #     print(subID + ' has trial that was never found')
            with open(file_low,'w') as csvfile:
                testwriter = csv.writer(csvfile,delimiter=',')
                for i in range(5):
                    row_save = [subID,control[con],
                                lives_all[subnum,i],treasure_all[subnum,i],score_all[subnum,i]]
                    testwriter.writerow(row_save)
            with open(file_high,'w') as csvfile:
                testwriter = csv.writer(csvfile,delimiter=',')
                for i in range(5,10):
                    row_save = [subID,control[con],
                                lives_all[subnum,i],treasure_all[subnum,i],score_all[subnum,i]]
                    testwriter.writerow(row_save)
            subnum += 1

# PLOT BOXPLOTS

# parameters
figure_size = (8,3.55) # sets the size of the figure in inches
labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
medianprops = dict(linewidth=2.5, color='black')
box_colors = ['#b71c1c','#ff6f00','#1b5e20','#006064','#1a237e','#b71c1c','#ff6f00','#1b5e20','#006064','#1a237e']
box_alpha = [0.5,0.5,0.5,0.5,0.5,None,None,None,None,None]

# -------------------------------------------------------------------
# Lives boxplot
# -------------------------------------------------------------------
fig_lives, ax_lives = plt.subplots(figsize=figure_size)
print(lives_all)
bp = ax_lives.boxplot(lives_all, notch=0, medianprops=medianprops, labels=labels)#, patch_artist=True,boxprops=dict(facecolor=color_combine, color=c))
plt.setp(bp['boxes'], color='black')
plt.setp(bp['whiskers'], color='black')
plt.setp(bp['fliers'], color='red', marker='+')
# Add a horizontal grid to the plot, but make it very light in color
# so we can use it for reading data values but not be distracting
ax_lives.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
               alpha=0.5)

# Hide these grid behind plot objects
ax_lives.set_axisbelow(True)
ax_lives.set_title('Aggregate Game Performance: Lives')#, fontsize=10, fontweight='bold')
ax_lives.set_xlabel('Experimental Condition')
ax_lives.set_ylabel('Number of lives at the end of the game')
# for label in (ax_lives.get_xticklabels() + ax_lives.get_yticklabels()):
#     label.set_fontsize(8)

medians = np.empty(10)
for i in range(10):
    box = bp['boxes'][i]
    boxX = []
    boxY = []
    for j in range(5):
        boxX.append(box.get_xdata()[j])
        boxY.append(box.get_ydata()[j])
    box_coords = np.column_stack([boxX, boxY])
    ax_lives.add_patch(Polygon(box_coords, facecolor=box_colors[i], alpha=box_alpha[i]))

x_coordinates = np.array([1,2,3,4,5,6,7,8,9,10])
y_coordinates = np.mean(lives_all,axis=0)
# print(np.mean(data_boxplot_low,axis=0))
ax_lives.plot(x_coordinates, y_coordinates, 'o',
             color='w', marker='o', markersize=7, markeredgecolor='black')#, linewidth=0)

# Seperate the stroke and control means and draw lines
plt.savefig('combined_lives.pdf')

# -------------------------------------------------------------------
# Treasure boxplot
# -------------------------------------------------------------------
fig_treas, ax_treas = plt.subplots(figsize=figure_size)
print(treasure_all)
bp = ax_treas.boxplot(treasure_all, notch=0, medianprops=medianprops, labels=labels)#, patch_artist=True,boxprops=dict(facecolor=color_combine, color=c))
plt.setp(bp['boxes'], color='black')
plt.setp(bp['whiskers'], color='black')
plt.setp(bp['fliers'], color='red', marker='+')
# Add a horizontal grid to the plot, but make it very light in color
# so we can use it for reading data values but not be distracting
ax_treas.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
               alpha=0.5)

# Hide these grid behind plot objects
ax_treas.set_axisbelow(True)
ax_treas.set_title('Aggregate Game Performance: Treasure')#, fontsize=10, fontweight='bold')
ax_treas.set_xlabel('Experimental Condition')
ax_treas.set_ylabel('Number of treasures at the end of the game')
# for label in (ax_treas.get_xticklabels() + ax_treas.get_yticklabels()):
#     label.set_fontsize(8)

medians = np.empty(10)
for i in range(10):
    box = bp['boxes'][i]
    boxX = []
    boxY = []
    for j in range(5):
        boxX.append(box.get_xdata()[j])
        boxY.append(box.get_ydata()[j])
    box_coords = np.column_stack([boxX, boxY])
    ax_treas.add_patch(Polygon(box_coords, facecolor=box_colors[i], alpha=box_alpha[i]))

x_coordinates = np.array([1,2,3,4,5,6,7,8,9,10])
y_coordinates = np.mean(treasure_all,axis=0)
# print(np.mean(data_boxplot_low,axis=0))
ax_treas.plot(x_coordinates, y_coordinates, 'o',
             color='w', marker='o', markersize=7, markeredgecolor='black')#, linewidth=0)

# Seperate the stroke and control means and draw lines
plt.savefig('combined_treas.pdf')

# -------------------------------------------------------------------
# Score boxplot
# -------------------------------------------------------------------
fig_score, ax_score = plt.subplots(figsize=figure_size)
print(score_all)
bp = ax_score.boxplot(score_all, notch=0, medianprops=medianprops, labels=labels)#, patch_artist=True,boxprops=dict(facecolor=color_combine, color=c))
plt.setp(bp['boxes'], color='black')
plt.setp(bp['whiskers'], color='black')
plt.setp(bp['fliers'], color='red', marker='+')
# Add a horizontal grid to the plot, but make it very light in color
# so we can use it for reading data values but not be distracting
ax_score.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
               alpha=0.5)

# Hide these grid behind plot objects
ax_score.set_axisbelow(True)
ax_score.set_title('Aggregate Game Performance: Score')#, fontsize=10, fontweight='bold')
ax_score.set_xlabel('Experimental Condition')
ax_score.set_ylabel('Score = #Lives*3 + #Treasures')
# for label in (ax_score.get_xticklabels() + ax_score.get_yticklabels()):
#     label.set_fontsize(8)

medians = np.empty(10)
for i in range(10):
    box = bp['boxes'][i]
    boxX = []
    boxY = []
    for j in range(5):
        boxX.append(box.get_xdata()[j])
        boxY.append(box.get_ydata()[j])
    box_coords = np.column_stack([boxX, boxY])
    ax_score.add_patch(Polygon(box_coords, facecolor=box_colors[i], alpha=box_alpha[i]))

x_coordinates = np.array([1,2,3,4,5,6,7,8,9,10])
y_coordinates = np.mean(score_all,axis=0)
# print(np.mean(data_boxplot_low,axis=0))
ax_score.plot(x_coordinates, y_coordinates, 'o',
             color='w', marker='o', markersize=7, markeredgecolor='black')#, linewidth=0)

# Seperate the stroke and control means and draw lines
plt.savefig('combined_score.pdf')


# plt.show()
