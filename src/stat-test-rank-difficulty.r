################################################################################
# INFO:
# This program performs the Kruskal-Wallis statistical test for rank order
# It was written for darpa HST data analysis
# In particular, it is used to perform statistical tests on the difficulty measure
# NOTE: Statistical results are published to output file created by sink()

################################################################################
# INSTRUCTIONS:
# Before running update the following
#   DIR location (line 52)
#   skill (novice/expert/all) to be analyzed (line 75)
#   data being analyzed (line 78) and
#   the sink file location (lines 88, 217, 327, 436, 544 and 653).
#
# If running from console on windows computer, use setwd to DIR location
#   setwd('/home/kpopovic/darpa_ws/src/darpa_data_analysis/src')
# For linux, use cd command to DIR location
#   cd /home/kpopovic/darpa_ws/src/darpa_data_analysis/src
#
# To run in the terminal, open R software and execute the following lines:
#   R
#   source('stat-test-rm.r')
library(rstatix)


# Specify DIR where data is located
DIR = '/home/milli/Desktop/darpa_dataanalysis/src'

# Parameters
skill = "all" # either "expert", "novice", or "all"


################################################################################
################################################################################
#                              Import Data                                     #
################################################################################
################################################################################

data_original = read.csv(paste(DIR,"raw_data_formatted","raw_data_formatted.csv",sep="/"))
if (skill=="expert"){
  data_control = subset(data_original, Lifetime>=1000)
} else if (skill=="novice"){
  data_control = subset(data_original, Lifetime<1000)
} else {
  data_control = data_original
}


# Define directory and file where to save the data to
sink(paste(DIR,"Stats","Difficulty",paste("difficulty",skill,"rank.txt",sep="-"),sep="/"))


# Select subset of the data for analysis
data_all = data_control
data_all = subset(data_all, Include_Difficulty=='True')
data_all = subset(data_all, Difficulty!='NaN')
data_all = subset(data_all, Density=='low')

data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)


cat("################################################################################ \n")
cat("wilcox_test for group difference for all data \n")
cat("################################################################################ \n")
pwc = wilcox_test(Difficulty ~ Control, paired = TRUE, p.adjust.method = "bonferroni", data = data_all)
print(pwc)


cat("\n")
cat("################################################################################ \n")
cat("############################## HIGH BUILDING DENSITY  ########################## \n")
cat("################################################################################ \n")

data_high = subset(data_control, Density=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

test = friedman.test(y=data_high$Difficulty, groups=data_high$Control, blocks=data_high$Subject)
print(test)
pwc = wilcox_test(Difficulty ~ Control, paired = TRUE, p.adjust.method = "bonferroni", data = data_high)
print(pwc)

cat("\n")
cat("################################################################################ \n")
cat("############################## LOW BUILDING DENSITY  ########################## \n")
cat("################################################################################ \n")

data_low = subset(data_control,Density=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

test = friedman.test(y=data_low$Difficulty, groups=data_low$Control, blocks=data_low$Subject)
print(test)
pwc = wilcox_test(Difficulty ~ Control, paired = TRUE, p.adjust.method = "bonferroni", data = data_low)
print(pwc)

