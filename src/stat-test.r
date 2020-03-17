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
#     > source('stat-test.r')
#     If running from console on windows computer, use setwd
#     for example - setwd('C:/Users/numur/Desktop/darpa_dataanalysis/src')
################################################################################
# COMMENTS:
# There are two equivalent ways of performing a repeated measures ANOVA:
#   1. aov(Score~(SupportLevel*SL_setnum) + Error(wid = .(Subject)/(SupportLevel*SL_setnum)))
#   2. ezANOVA(data,dv=Score,wid=wid = .(Subject),within = .(SupportLevel,SL_setnum),between = NULL, detailed = TRUE)
# The second also tests for sphericity, but sometimes gives errors if nothing is significant.
################################################################################

options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())
library(ez) # loads the ez package and dependencies

DIR = 'C:/Users/numur/Desktop/darpa_dataanalysis/src'  #'/home/murpheylab/catkin_ws/src/VR_exp_ROS/darpa_dataanalysis/src' 

# Skill>6.69 
# Lifetime<600 score significant high autonomy
# Lifetime>600  significant  autonomy
data_control_original = read.csv(paste(DIR,paste("control","all.csv",sep="_"),sep="/"))
data_control= subset(data_control_original, Lifetime<600)
data_method_original = read.csv(paste(DIR,paste("method","all.csv",sep="_"),sep="/"))
data_method= subset(data_method_original, Lifetime<600)
data_autonomy_original = read.csv(paste(DIR,paste("autonomy","all.csv",sep="_"),sep="/"))
data_autonomy= subset(data_autonomy_original, Lifetime<600)

################################################################################
################################################################################
#               CONTROL
################################################################################
################################################################################

sink(paste(DIR,paste("control","stats1.txt",sep="-"),sep="/"))
data_all = subset(data_control,Uselow==1 & Usehigh==1)
# data = read.csv(paste(DIR,paste("performance","seperate.csv",sep="_"),sep="/"))
# attach(data)
#################################################
cat("\n")
cat("################################################## COMBINED ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_all,Lives,wid = .(Subject),within = .(Control,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_all,Treasure,wid = .(Subject),within = .(Control,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_all,Score,wid = .(Subject),within = .(Control,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data)


data_high = subset(data_control, Complexity=='high' & Usehigh==1)
  #read.csv(paste(DIR,paste("control","high.csv",sep="_"),sep="/"))
# attach(data_high)
#################################################
cat("\n")
cat("################################################## HIGH ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_high,Lives,wid = .(Subject),within = .(Control),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_high,Treasure,wid = .(Subject),within = .(Control),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_high,Score,wid = .(Subject),within = .(Control),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data_high)

data_low = subset(data_control,Complexity=='low' & Uselow==1)
# read.csv(paste(DIR,paste("control","low.csv",sep="_"),sep="/"))
# attach(data_low)
#################################################
cat("\n")
cat("################################################## LOW ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_low,Lives,wid = .(Subject),within = .(Control),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_low,Treasure,wid = .(Subject),within = .(Control),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_low,Score,wid = .(Subject),within = .(Control),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data_low)

################################################################################
################################################################################
#             Method
################################################################################
################################################################################

sink(paste(DIR,paste("method","stats1.txt",sep="-"),sep="/"))
data_all = subset(data_method,Uselow==1 & Usehigh==1)
# attach(data)
#################################################
cat("\n")
cat("################################################## COMBINED ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_all,Lives,wid = .(Subject),within = .(Method,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_all,Treasure,wid = .(Subject),within = .(Method,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_all,Score,wid = .(Subject),within = .(Method,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data)

data_high = subset(data_method,Complexity=='high' & Usehigh==1)
# data_high = read.csv(paste(DIR,paste("method","high.csv",sep="_"),sep="/"))
# attach(data_high)
#################################################
cat("\n")
cat("################################################## HIGH ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_high,Lives,wid = .(Subject),within = .(Method),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_high,Treasure,wid = .(Subject),within = .(Method),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_high,Score,wid = .(Subject),within = .(Method),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data_high)

data_low = subset(data_method,Complexity=='low' & Uselow==1)
# data_low = read.csv(paste(DIR,paste("method","low.csv",sep="_"),sep="/"))
# attach(data_low)
#################################################
cat("\n")
cat("################################################## LOW ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_low,Lives,wid = .(Subject),within = .(Method),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_low,Treasure,wid = .(Subject),within = .(Method),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_low,Score,wid = .(Subject),within = .(Method),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data_low)


################################################################################
################################################################################
#             Method
################################################################################
################################################################################


sink(paste(DIR,paste("autonomy","stats1.txt",sep="-"),sep="/"))
data_all = subset(data_autonomy,Uselow==1 & Usehigh==1)
# attach(data)
#################################################
cat("\n")
cat("################################################## COMBINED ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_all,Lives,wid = .(Subject),within = .(Autonomy,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_all,Treasure,wid = .(Subject),within = .(Autonomy,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_all,Score,wid = .(Subject),within = .(Autonomy,Complexity),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data)

data_high = subset(data_autonomy,Complexity=='high' & Usehigh==1)
# data_high = read.csv(paste(DIR,paste("autonomy","high.csv",sep="_"),sep="/"))
# attach(data_high)
#################################################
cat("\n")
cat("################################################## HIGH ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_high,Lives,wid = .(Subject),within = .(Autonomy),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_high,Treasure,wid = .(Subject),within = .(Autonomy),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_high,Score,wid = .(Subject),within = .(Autonomy),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data_high)

data_low = subset(data_autonomy,Complexity=='low' & Uselow==1)
# data_low = read.csv(paste(DIR,paste("autonomy","low.csv",sep="_"),sep="/"))
# attach(data_low)
#################################################
cat("\n")
cat("################################################## LOW ################################### \n")
cat("##################################################Lives############ \n")
mod.ez<-ezANOVA(data_low,Lives,wid = .(Subject),within = .(Autonomy),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Treasure############ \n")
mod.ez<-ezANOVA(data_low,Treasure,wid = .(Subject),within = .(Autonomy),between = NULL, detailed = TRUE)
print(mod.ez)
cat("##################################################Score############ \n")
mod.ez<-ezANOVA(data_low,Score,wid = .(Subject),within = .(Autonomy),between = NULL, detailed = TRUE)
print(mod.ez)
# detach(data_low)
