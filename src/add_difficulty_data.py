#!/usr/bin/env python

# This script calls dfficulty class to updated the pefromance.csv file with the
# difficulty rating score. NOTE: This funciton needs to be called separately
# from the save_data.py as there's a chance that missing_bags will need to be
# analyzed using performance_subscriber. Only after all the performance data is
# complete one should run the add_difficulty_data script to append the
# difficulty rating column.

from utils.difficulty import update_performance
import os

drFileLoc = 'raw_data/difficulty_rating.csv'
pFileLoc = 'raw_data/'
tiFileLoc = 'temp_delete/'
tiFolderLoc = 'HST_data_local/trial_info/'
if not os.path.exists('temp_delete/trial_info_dr/'):
    os.makedirs('temp_delete/trial_info_dr/')
tiFolderLocNew = 'temp_delete/trial_info_dr/'

update_performance(pFileLoc, drFileLoc, tiFileLoc, tiFolderLoc, tiFolderLocNew)
