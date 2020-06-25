# Imports
import csv
import numpy as np
import matplotlib.pyplot as plt

# Set filenames for getting raw data
file_subdata = "subject_info.csv" # contains subject info from the questionaire
        # like the number of hours spent playing video games over their lifetime


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
plt.savefig('aggregateplots/'+'method_feedback.pdf')

plt.figure(51,dpi=150)
width = 0.5
ind = np.arange(3)
print(autonomy_count)
p1 = plt.bar(ind,autonomy_count,width)
plt.ylabel('Number of Participants')
plt.title('Participant feedback for preferred level of autonomy')
labels = ('direct','shared','fully autonomous')
plt.xticks(ind, labels, rotation=15)
plt.savefig('aggregateplots/'+'autonomy_feedback.pdf')
