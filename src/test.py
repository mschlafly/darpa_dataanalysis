
# Importing library
import csv
import pandas as pd


print('Panda version is ', pd.__version__)
# data to be written row-wise in csv fil
# data = [['Geeks'], [4], ['geeks !']]
# data = [[4], [5], [6]]
data = [4, 5, 6]

# opening the csv file in 'w+' mode
file = open('test.csv', 'w+')

# writing the data into the file
with file:
    write = csv.writer(file)
    write.writerow(data)
    # write.writerows(data)
