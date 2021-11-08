#!/usr/bin/env python

# This script calls dfficulty class to updated the pefromance.csv file with the
# difficulty rating score. NOTE: This funciton needs to be called separately
# from the save_data.py as there's a chance that missing_bags will need to be
# analyzed using performance_subscriber. Only after all the performance data is
# complete one should run the add_difficulty_data script to append the
# difficulty rating column.

from difficulty import update_performance

drFileLoc = 'hst_info/difficulty_rating.csv'
pFileLoc = 'performance_info/'
tiFileLoc = 'hst_info/'
tiFolderLoc = 'trial_info/'
tiFolderLocNew = 'trial_info_dr/'

update_performance(pFileLoc, drFileLoc, tiFileLoc, tiFolderLoc, tiFolderLocNew)
