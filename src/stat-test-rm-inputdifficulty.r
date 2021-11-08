################################################################################
# INFO:
# This program performs ANOVA repeated measures statistical tests
# It was written for darpa HST data analysis
# NOTE: Statistical results are published to output file created by sink()

################################################################################
# DEPENDENCIES:
# The exANOVA function in the ez package is used to perform statistical tests.
# The rstatix package is used for the %>% function.
#
# This install packages run the following commands
#   install.package("ez")
#   install.package("rstatix")
#
# Fore more info please see README.md file.

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

################################################################################
# COMMENTS:
# To test a different metric, Ctrl replace the name of the metric
#
# There are two equivalent ways of performing a repeated measures ANOVA:
#   1. aov(Difficulty~(SupportLevel*SL_setnum)
#                 + Error(wid = .(Subject)/(SupportLevel*SL_setnum)))
#   2. ezANOVA(data,
#              dv = Difficulty,
#              wid = .(Subject),
#              within = .(SupportLevel,SL_setnum),
#              between = NULL,
#              type = 2,
#              detailed = TRUE)
#
# NOTE: The second way tests for sphericity, but sometimes gives errors if
# nothing is significant.

################################################################################
################################################################################
#                               Package Import                                 #
################################################################################
################################################################################

options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())

# loads packages and dependencies
library(ez)
library(rstatix)

################################################################################
################################################################################
#               Specify directories and metric to be analyzed                  #
################################################################################
################################################################################

# Specify DIR where data is located
DIR = 'C:/Users/numur/Desktop/darpa_data_analysis/src' 
# DIR = '/home/kpopovic/darpa_ws/src/darpa_data_analysis/src'

# Parameters
skill = "all" # either "expert", "novice", or "all"

# Metric to be analyzed:
#   'Input' for section line 110 - line 277
#   'Difficulty' for the rest of the file
# ctrl-f in order to replace all the instances of desired metric

################################################################################
################################################################################
#                              Import Data                                     #
################################################################################
################################################################################

data_control_original = read.csv(paste(DIR,paste("performance_stat",".csv",sep=""),sep="/"))
data_control_original = subset(data_control_original, Score!=0) # remove incorrectly added trials
if (skill=="expert"){
  data_control = subset(data_control_original, Lifetime>999)
} else if (skill=="novice"){
  data_control = subset(data_control_original, Lifetime<999)
} else {
  data_control = data_control_original
}

data_method_direct = subset(data_control, Control!='sharedergodic' & Control!='autoergodic')
data_method_shared = subset(data_control, Control!='directergodic' & Control!='autoergodic')
data_method_auto = subset(data_control, Control!='directergodic' & Control!='sharedergodic')
data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')
data_inputs = subset(data_control, Control!='none' & Control!='autoergodic')

################################################################################
################################################################################
#                                   Inputs                                     #
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("inputs","Input",skill,"rm.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("##################################### ALL ###################################### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis
data_all = subset(data_inputs,Uselow==1 & Usehigh==1)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Check assumptions
# Assumption 1: Independence of samples - satisfied by the experimental set-up
# Assumption 2: Data is normally distributed - check visually using boxplots or
#               histograms AND use the shapiro test (if the null hypothesis is
#               violated p<.05, it is not normally distributed)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_all %>%
  group_by(Complexity,Control) %>%
  shapiro_test(Input)
print(normality)

# Assumption 3: Sphericity/Equal variances between treatments - ezANOVA
#               performs this test. If the null hypothesis is violated (p<.05
#               for any particular factor), use a corrected p-value, possibly
#               the Greenhouse-Geisser p[GG]

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_all,Input,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,Input,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

# If a factor is significant, that means that at least one of the groups is
# different from the others. But you still do not know which group is different.
# T-tests can help.

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")

posthoc<-pairwise.t.test(data_all$Input,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# Compare all group/factor combinations for putting asterisks on plots

cat("\n")
cat("################################################################################ \n")
cat("Comparison for marking plots with asterisks \n")
cat("################################################################################ \n")

data_all$combo <- paste(data_all$Control,data_all$Complexity)
cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_all$Input,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# If there is an interaction effect between two factors (in this case complexity and control),
# analyze trends within each group separately

cat("\n")
cat("################################################################################ \n")
cat("##################################### HIGH ##################################### \n")
cat("################################################################################ \n")

data_high = subset(data_inputs, Complexity=='high' & Usehigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(Input)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")

if (skill=="all") {
  mod.ez<-ezANOVA(data_high,Input,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,Input,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")

posthoc<-pairwise.t.test(data_high$Input,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("################################################################################ \n")
cat("##################################### LOW ###################################### \n")
cat("################################################################################ \n")
data_low = subset(data_inputs,Complexity=='low' & Uselow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(Input)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")

if (skill=="all") {
  mod.ez<-ezANOVA(data_low,Input,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,Input,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")

posthoc<-pairwise.t.test(data_low$Input,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

################################################################################
################################################################################
#                                 CONTROL                                      #
################################################################################
################################################################################

# Define directory and file where to save the data to
sink(paste(DIR,"stattests",paste("control","Difficulty",skill,"rm.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("##################################### ALL ###################################### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis
data_all = subset(data_control,Uselow==1 & Usehigh==1)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Check assumptions
# Assumption 1: Independence of samples - satisfied by the experimental set-up
# Assumption 2: Data is normally distributed - check visually using boxplots or
#               histograms AND use the shapiro test (if the null hypothesis is
#               violated p<.05, it is not normally distributed
cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_all %>%
  group_by(Complexity,Control) %>%
  shapiro_test(Difficulty)
print(normality)

# Assumption 3: Sphericity/Equal variances between treatments - ezANOVA
#               performs this test if the null hypothesis is violated p<.05 for
#               any particular factor, use a corrected p-value, possibly the
#               Greenhouse-Geisser p[GG]
cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_all,Difficulty,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,Difficulty,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

# If a factor is significant, that means that at least one of the groups is
# different from the others. But you still do not know which group is different
# T-tests can help.
cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_all$Difficulty,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
# Compare all group/factor combinations for putting asterisks on plots
cat("\n")
cat("################################################################################ \n")
cat("Comparison for marking plots with asterisks \n")
cat("################################################################################ \n")
data_all$combo <- paste(data_all$Control,data_all$Complexity)
posthoc<-pairwise.t.test(data_all$Difficulty,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# If there is an interaction effect between two factors (in this case complexity
# and control), analyze trends within each group separately.

cat("\n")
cat("################################################################################ \n")
cat("##################################### HIGH ##################################### \n")
cat("################################################################################ \n")

data_high = subset(data_control, Complexity=='high' & Usehigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(Difficulty)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_high,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_high$Difficulty,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("################################################################################ \n")
cat("##################################### LOW ###################################### \n")
cat("################################################################################ \n")
data_low = subset(data_control,Complexity=='low' & Uselow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(Difficulty)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_low,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_low$Difficulty,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# ################################################################################
# ################################################################################
# #                               METHOD - DIRECT                                #
# ################################################################################
# ################################################################################
# 
# sink(paste(DIR,"stattests",paste("directmethod","Difficulty",skill,"rm.txt",sep="-"),sep="/"))
# 
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### ALL ###################################### \n")
# cat("################################################################################ \n")
# 
# data_all = subset(data_method_direct,Uselow==1 & Usehigh==1)
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_all %>%
#   group_by(Complexity,Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_all,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_all,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_all$Difficulty,
#                          data_all$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Difficulty,
#                          data_all$combo,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### HIGH ##################################### \n")
# cat("################################################################################ \n")
# 
# data_high = subset(data_method_direct, Complexity=='high' & Usehigh==1)
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_high %>%
#   group_by(Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_high,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_high,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_high$Difficulty,
#                          data_high$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### LOW ###################################### \n")
# cat("################################################################################ \n")
# data_low = subset(data_method_direct,Complexity=='low' & Uselow==1)
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_low %>%
#   group_by(Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_low,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_low,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_low$Difficulty,
#                          data_low$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# 
# ################################################################################
# ################################################################################
# #                               METHOD - SHARED                                #
# ################################################################################
# ################################################################################
# 
# sink(paste(DIR,"stattests",paste("sharedmethod","Difficulty",skill,"rm.txt",sep="-"),sep="/"))
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### ALL ###################################### \n")
# cat("################################################################################ \n")
# 
# data_all = subset(data_method_shared,Uselow==1 & Usehigh==1)
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_all %>%
#   group_by(Complexity,Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_all,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_all,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_all$Difficulty,
#                          data_all$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Difficulty,
#                          data_all$combo,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### HIGH ##################################### \n")
# cat("################################################################################ \n")
# 
# data_high = subset(data_method_shared, Complexity=='high' & Usehigh==1)
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_high %>%
#   group_by(Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_high,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_high,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_high$Difficulty,
#                          data_high$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### LOW ###################################### \n")
# cat("################################################################################ \n")
# data_low = subset(data_method_shared,Complexity=='low' & Uselow==1)
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_low %>%
#   group_by(Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_low,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_low,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_low$Difficulty,
#                          data_low$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# 
# ################################################################################
# ################################################################################
# #                               METHOD - AUTO                                  #
# ################################################################################
# ################################################################################
# 
# sink(paste(DIR,"stattests",paste("automethod","Difficulty",skill,"rm.txt",sep="-"),sep="/"))
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### ALL ###################################### \n")
# cat("################################################################################ \n")
# 
# data_all = subset(data_method_auto,Uselow==1 & Usehigh==1)
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_all %>%
#   group_by(Complexity,Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_all,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_all,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_all$Difficulty,
#                          data_all$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Difficulty,
#                          data_all$combo,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### HIGH ##################################### \n")
# cat("################################################################################ \n")
# 
# data_high = subset(data_method_auto, Complexity=='high' & Usehigh==1)
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_high %>%
#   group_by(Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_high,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_high,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_high$Difficulty,
#                          data_high$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# cat("\n")
# cat("################################################################################ \n")
# cat("##################################### LOW ###################################### \n")
# cat("################################################################################ \n")
# data_low = subset(data_method_auto,Complexity=='low' & Uselow==1)
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for normality: Shapiro test\n")
# cat("################################################################################ \n")
# normality = data_low %>%
#   group_by(Control) %>%
#   shapiro_test(Difficulty)
# print(normality)
# 
# cat("\n")
# cat("################################################################################ \n")
# cat("Test for Sphericity: ezANOVA \n")
# cat("################################################################################ \n")
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_low,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_low,Difficulty,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# cat("################################################################################ \n")
# cat("T-test for group difference \n")
# cat("################################################################################ \n")
# posthoc<-pairwise.t.test(data_low$Difficulty,
#                          data_low$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)

################################################################################
################################################################################
#                               AUTONOMY                                       #
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("autonomy","Difficulty",skill,"rm.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("##################################### ALL ###################################### \n")
cat("################################################################################ \n")

data_all = subset(data_autonomy,Uselow==1 & Usehigh==1)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_all %>%
  group_by(Complexity,Control) %>%
  shapiro_test(Difficulty)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_all,Difficulty,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,Difficulty,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_all$Difficulty,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
data_all$combo <- paste(data_all$Control,data_all$Complexity)
posthoc<-pairwise.t.test(data_all$Difficulty,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("################################################################################ \n")
cat("##################################### HIGH ##################################### \n")
cat("################################################################################ \n")

data_high = subset(data_autonomy, Complexity=='high' & Usehigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(Difficulty)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_high,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_high$Difficulty,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("################################################################################ \n")
cat("##################################### LOW ###################################### \n")
cat("################################################################################ \n")
data_low = subset(data_autonomy,Complexity=='low' & Uselow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("################################################################################ \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(Difficulty)
print(normality)

cat("\n")
cat("################################################################################ \n")
cat("Test for Sphericity: ezANOVA \n")
cat("################################################################################ \n")
if (skill=="all") {
  mod.ez<-ezANOVA(data_low,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,Difficulty,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("################################################################################ \n")
posthoc<-pairwise.t.test(data_low$Difficulty,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
