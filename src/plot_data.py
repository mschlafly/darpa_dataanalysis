import csv
import numpy as np
import matplotlib.pyplot as plt


file = "performance.csv"

list_of_complete_datasets =[1,7,8,9,11,13,14,15,17,18,20,21,22,23,24,25,26,27,28,29,30,31,32,33,36,37,38]
list_of_complete_datasets =[1,7,8,9,11,13,14,15,18,20,21,22,24,25,26,27,28,29,30,31,32,33,36,37,38]
num_combined = np.zeros(10, dtype=int)
player_lives_all = np.zeros((10,len(list_of_complete_datasets)))
treasures_found_all = np.zeros((10,len(list_of_complete_datasets)))

environments = ['low','high']
control = ['none','waypoint','directergodic','sharedergodic','autoergodic']


with open(file,'r') as csvfile:
    data = csv.reader(csvfile,delimiter=',')
    for row in data:
        if row[2]==environments[0]:
            for i in range(len(control)):
                if row[1]==control[i]:
                    player_lives_all[i,num_combined[i]] = row[3]
                    treasures_found_all[i,num_combined[i]] = row[4]
                    num_combined[i] += 1
        else:
            for i in range(len(control)):
                if row[1]==control[i]:
                    player_lives_all[i+5,num_combined[i+5]] = row[3]
                    treasures_found_all[i+5,num_combined[i+5]] = row[4]
                    num_combined[i+5] += 1

width = 0.5
ind = np.arange(10)
plt.figure(100)
player_lives_mean = np.zeros(10)
player_lives_std = np.zeros(10)
treasures_found_mean = np.zeros(10)
treasures_found_std = np.zeros(10)
for i in range(10):
    player_lives_mean[i] = np.mean(player_lives_all[i,0:num_combined[i]])
    print(num_combined[i],player_lives_all[i,0:num_combined[i]],player_lives_mean[i])
    player_lives_std[i] = np.std(player_lives_all[i,0:num_combined[i]])/np.sqrt(num_combined[i])
    treasures_found_mean[i] = np.mean(treasures_found_all[i,0:num_combined[i]])
    treasures_found_std[i] = np.std(treasures_found_all[i,0:num_combined[i]])/np.sqrt(num_combined[i])

p1 = plt.bar(ind,player_lives_mean,width,yerr=player_lives_std)
p2 = plt.bar(ind,treasures_found_mean,width,bottom=player_lives_mean,yerr=treasures_found_std)
plt.ylabel('Score')
plt.title('Aggregate Game Performance')
labels = ('LN','LW','LD','LS','LA','HN','HW','HD','HS','HA')
plt.xticks(ind, labels)
plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
plt.savefig('combined_performance.pdf')

plt.show()
