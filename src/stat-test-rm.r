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
#     > source('stat-test-rm.r')
#     If running from console on windows computer, use setwd
#     for example - setwd('C:/Users/numur/Desktop/darpa_dataanalysis/src')
################################################################################
# COMMENTS:
# - To test a different metric, Ctrl replace the name of the metric
# There are two equivalent ways of performing a repeated measures ANOVA:
#   1. aov(Zscore~(SupportLevel*SL_setnum) + Error(wid = .(Subject)/(SupportLevel*SL_setnum)))
#   2. ezANOVA(data,dv=Zscore,wid=wid = .(Subject),within = .(SupportLevel,SL_setnum),between = NULL, type = 2, detailed = TRUE)
# The second also tests for sphericity, but sometimes gives errors if nothing is significant.
################################################################################

options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())

# loads packages and dependencies
library(ez)
library(rstatix) # for the %>% function

# parameters
DIR = 'C:/Users/numur/Desktop/darpa_data_analysis/src'
# DIR = '/home/mschlafly/Desktop/darpa_data_analysis/src'
skill = "expert" # either "expert", "novice", or "all"

################################################################################
################################################################################
#               Import Data
################################################################################
################################################################################
data_control_original = read.csv(paste(DIR,paste("RR_stat",".csv",sep=""),sep="/"))
data_control_original = subset(data_control_original, Zscore!=0) # remove incorrectly added trials
if (skill=="expert"){
  # data_control = subset(data_control_original, Lifetime>999)
  data_control = subset(data_control_original, Expertise=='expert')
} else if (skill=="novice"){
  # data_control = subset(data_control_original, Lifetime<999)
  data_control = subset(data_control_original, Expertise=='novice')
} else {
  data_control = data_control_original
}
data_method_direct = subset(data_control, Control!='sharedergodic' & Control!='autoergodic')
data_method_shared = subset(data_control, Control!='directergodic' & Control!='autoergodic')
data_method_auto = subset(data_control, Control!='directergodic' & Control!='sharedergodic')
data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')

################################################################################
################################################################################
#               CONTROL
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("control","Zscore",skill,"rm.txt",sep="-"),sep="/"))
cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_all = subset(data_control,Uselow==1 & Usehigh==1)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Check assumptions
# Assumption 1: Independence of samples - satisfied by the experimental set-up
# Assumption 2: Data is normally distributed - check visually using boxplots or histograms
#               AND use the shapiro test (if the null hypothesis is violated p<.05, it is not
#               normally distributed
cat("Test for normality \n")
print(data_all %>%
        group_by(Complexity,Control))
normality = data_all %>%
  group_by(Complexity,Control) %>%
  shapiro_test(Zscore)
print(normality)

# Assumption 3: Sphericity/Equal variances between treatments - ezANOVA performs this test
#               if the null hypothesis is violated p<.05 for any particular factor, use a corrected
#               p-value, possibly the Greenhouse-Geisser p[GG]

if (skill=="all") {
  mod.ez<-ezANOVA(data_all,Zscore,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,Zscore,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

# If a factor is significant, that means that at least one of the groups is different from the others
# But you still do not know which group is different. T-tests can help.
posthoc<-pairwise.t.test(data_all$Zscore,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
# Compare all group/factor combinations for putting asterisks on plots
data_all$combo <- paste(data_all$Control,data_all$Complexity)
posthoc<-pairwise.t.test(data_all$Zscore,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# If there is an interaction effect between two factors (in this case complexity and control),
# analyze trends within each group seperately

cat("\n")
cat("########################################################################### \n")
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

data_high = subset(data_control, Complexity=='high' & Usehigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(Zscore)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_high,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_high$Zscore,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")
data_low = subset(data_control,Complexity=='low' & Uselow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(Zscore)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_low,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_low$Zscore,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# ################################################################################
# ################################################################################
# #               METHOD - DIRECT
# ################################################################################
# ################################################################################
# 
# sink(paste(DIR,"stattests",paste("directmethod","Zscore",skill,"rm.txt",sep="-"),sep="/"))
# 
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### ALL ################################### \n")
# cat("########################################################################### \n")
# 
# data_all = subset(data_method_direct,Uselow==1 & Usehigh==1)
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_all %>%
#   group_by(Complexity,Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_all,Zscore,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_all,Zscore,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# posthoc<-pairwise.t.test(data_all$Zscore,
#                          data_all$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Zscore,
#                          data_all$combo,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### HIGH ################################## \n")
# cat("########################################################################### \n")
# 
# data_high = subset(data_method_direct, Complexity=='high' & Usehigh==1)
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_high %>%
#   group_by(Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_high,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_high,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# posthoc<-pairwise.t.test(data_high$Zscore,
#                          data_high$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### LOW ################################### \n")
# cat("########################################################################### \n")
# data_low = subset(data_method_direct,Complexity=='low' & Uselow==1)
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_low %>%
#   group_by(Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_low,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_low,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# posthoc<-pairwise.t.test(data_low$Zscore,
#                          data_low$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# 
# ################################################################################
# ################################################################################
# #               METHOD - SHARED
# ################################################################################
# ################################################################################
# 
# sink(paste(DIR,"stattests",paste("sharedmethod","Zscore",skill,"rm.txt",sep="-"),sep="/"))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### ALL ################################### \n")
# cat("########################################################################### \n")
# 
# data_all = subset(data_method_shared,Uselow==1 & Usehigh==1)
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_all %>%
#   group_by(Complexity,Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_all,Zscore,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_all,Zscore,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# posthoc<-pairwise.t.test(data_all$Zscore,
#                          data_all$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Zscore,
#                          data_all$combo,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### HIGH ################################## \n")
# cat("########################################################################### \n")
# 
# data_high = subset(data_method_shared, Complexity=='high' & Usehigh==1)
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_high %>%
#   group_by(Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_high,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_high,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# posthoc<-pairwise.t.test(data_high$Zscore,
#                          data_high$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### LOW ################################### \n")
# cat("########################################################################### \n")
# data_low = subset(data_method_shared,Complexity=='low' & Uselow==1)
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_low %>%
#   group_by(Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_low,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_low,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# posthoc<-pairwise.t.test(data_low$Zscore,
#                          data_low$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# 
# ################################################################################
# ################################################################################
# #               METHOD - AUTO
# ################################################################################
# ################################################################################
# 
# sink(paste(DIR,"stattests",paste("automethod","Zscore",skill,"rm.txt",sep="-"),sep="/"))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### ALL ################################### \n")
# cat("########################################################################### \n")
# 
# data_all = subset(data_method_auto,Uselow==1 & Usehigh==1)
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_all %>%
#   group_by(Complexity,Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_all,Zscore,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_all,Zscore,
#                   wid = .(Subject),
#                   within = .(Control,Complexity),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# posthoc<-pairwise.t.test(data_all$Zscore,
#                          data_all$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Zscore,
#                          data_all$combo,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### HIGH ################################## \n")
# cat("########################################################################### \n")
# 
# data_high = subset(data_method_auto, Complexity=='high' & Usehigh==1)
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_high %>%
#   group_by(Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_high,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_high,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# 
# posthoc<-pairwise.t.test(data_high$Zscore,
#                          data_high$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### LOW ################################### \n")
# cat("########################################################################### \n")
# data_low = subset(data_method_auto,Complexity=='low' & Uselow==1)
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# cat("Test for normality \n")
# normality = data_low %>%
#   group_by(Control) %>%
#   shapiro_test(Zscore)
# print(normality)
# 
# if (skill=="all") {
#   mod.ez<-ezANOVA(data_low,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = Expertise, type = 3, detailed = TRUE)
# } else {
#   mod.ez<-ezANOVA(data_low,Zscore,
#                   wid = .(Subject),
#                   within = .(Control),
#                   between = NULL, type = 2, detailed = TRUE)
# }
# print(mod.ez)
# posthoc<-pairwise.t.test(data_low$Zscore,
#                          data_low$Control,
#                          paired = TRUE,
#                          p.adjust.method = "bonferroni")
# print(posthoc)

################################################################################
################################################################################
#               AUTONOMY
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("autonomy","Zscore",skill,"rm.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

data_all = subset(data_autonomy,Uselow==1 & Usehigh==1)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_all %>%
  group_by(Complexity,Control) %>%
  shapiro_test(Zscore)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_all,Zscore,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,Zscore,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_all$Zscore,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
data_all$combo <- paste(data_all$Control,data_all$Complexity)
posthoc<-pairwise.t.test(data_all$Zscore,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

data_high = subset(data_autonomy, Complexity=='high' & Usehigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(Zscore)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_high,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_high$Zscore,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")
data_low = subset(data_autonomy,Complexity=='low' & Uselow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(Zscore)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_low,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,Zscore,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)
posthoc<-pairwise.t.test(data_low$Zscore,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
