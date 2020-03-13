################################################################################
# This program performs ANOVA repeated measures statistical tests for darpa HST data.
# Statistical results are published to output file created by sink()

# The exANOVA function in the ez package is used to perform statistical tests.
# This package and dependencies will need to be installed.
# To install run : install.package("ez")
################################################################################
# INSTRUCTIONS:
#     Change DIR, data, and the sink file before running in the terminal
#     To run in the terminal
#     $ R
#     Then run the script by executing the following
#     > source('stats-test.r')
#     If running from console on windows computer, use setwd
#     for example - setwd('C:/Users/numur/Desktop/HapticMaster/FrequencyAnalysis')
################################################################################
# COMMENTS:
# There are two equivalent ways of performing a repeated measures ANOVA:
#   1. aov(Score~(SupportLevel*SL_setnum) + Error(Subject/(SupportLevel*SL_setnum)))
#   2. ezANOVA(data,dv=Score,wid=Subject,within = .(SupportLevel,SL_setnum),between = NULL, detailed = TRUE)
# The second also tests for sphericity, but sometimes gives errors if nothing is significant.
################################################################################

options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())
library(ez) # loads the ez package and dependencies

DIR = '/home/murpheylab/catkin_ws/src/VR_exp_ROS/darpa_dataanalysis/src' 
sink(paste(DIR,paste("all","stats.txt",sep="-"),sep="/"))

data = read.csv(paste(DIR,paste("metrics","all.csv",sep="_"),sep="/"))
attach(data)
#################################################
cat("\n")
cat("################################################## COMBINED ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data,Lives,Subject,within = .(Control,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data,Treasure,Subject,within = .(Control,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data,Score,Subject,within = .(Control,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
detach(data)

