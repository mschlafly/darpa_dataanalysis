################################################################################
# This program performs ANOVA repeated measures statistical tests for darpa HST data.
# Statistical results are published to output file created by sink()
# In particular, it is used to perform statistical tests on the RRinterval and RR
# outcome measures. To test RR, ctl find replace "RR" with "Score" and 
# "RR_raw_formatted.csv" with "raw_data_formatted.csv"

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
#   1. aov(Mean~(SupportLevel*SL_setnum) + ERRor(wid = .(Subject)/(SupportLevel*SL_setnum)))
#   2. ezANOVA(data,dv=Mean,wid=wid = .(Subject),within = .(SupportLevel,SL_setnum),between = NULL, type = 2, detailed = TRUE)
# The second also tests for sphericity, but sometimes gives eRRors if nothing is significant.
################################################################################

options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())

# loads packages and dependencies
library(ez)
library(rstatix) # for the %>% function


library(lme4) # lmer
# library(nlme) #lme


# parameters
DIR = 'C:/Users/milli/OneDrive/Documents/darpa_dataanalysis/src'
skill = "expert" # either "expert", "novice", or "all"

################################################################################
################################################################################
#               Import Data
################################################################################
################################################################################
data_control_original = read.csv(paste(DIR,"raw_data_formatted","RR_raw_formatted.csv",sep="/"))

if (skill=="expert"){
  data_control = subset(data_control_original, Lifetime>999)
  # data_control = subset(data_control_original, Expertise=='expert')
} else if (skill=="novice"){
  data_control = subset(data_control_original, Lifetime<999)
  # data_control = subset(data_control_original, Expertise=='novice')
} else {
  data_control = data_control_original
}

data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')

################################################################################
################################################################################
#               CONTROL
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","RR",paste("control","RR",skill,"rm.txt",sep="-"),sep="/"))
cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Complexity=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis
data_all = subset(data_control,Alllow==1 & Allhigh==1)
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
  shapiro_test(RR)
print(normality)

# Assumption 3: Sphericity/Equal variances between treatments - ezANOVA performs this test
#               if the null hypothesis is violated p<.05 for any particular factor, use a coRRected
#               p-value, possibly the Greenhouse-Geisser p[GG]

if (skill=="all") {
  mod.ez<-ezANOVA(data_all,RR,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

# If a factor is significant, that means that at least one of the groups is different from the others
# But you still do not know which group is different. T-tests can help.
data_none = subset(data_all, Control=='none')
data_wp = subset(data_all, Control=='waypoint')
data_user = subset(data_all, Control=='directergodic')
data_shared = subset(data_all, Control=='sharedergodic')
data_auto = subset(data_all, Control=='autoergodic')

posthoc<-t.test(data_none$RR,data_wp$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_none$RR,data_user$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_none$RR,data_shared$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_none$RR,data_auto$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_wp$RR,data_user$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_wp$RR,data_shared$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_wp$RR,data_auto$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_user$RR,data_shared$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_user$RR,data_auto$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)
posthoc<-t.test(data_shared$RR,data_auto$RR,
                paired = TRUE,detailed = TRUE)
print(posthoc)
print("Adjusted p-val bonferroni")
print(posthoc$p.value*10)

posthoc<-pairwise.t.test(data_all$RR,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni",detailed = TRUE)
print(posthoc)

# Compare all group/factor combinations for putting asterisks on plots
data_all$combo <- paste(data_all$Control,data_all$Complexity)
posthoc<-pairwise.t.test(data_all$RR,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# If there is an interaction effect between two factors (in this case complexity and control),
# analyze trends within each group seperately

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

data_high = subset(data_control, Complexity=='high' & Allhigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(RR)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_high,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_high$RR,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("########################## LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
data_low = subset(data_control,Complexity=='low' & Alllow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(RR)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_low,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_low$RR,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

################################################################################
################################################################################
#               AUTONOMY
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","RR",paste("autonomy","RR",skill,"rm.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Complexity=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

data_all = subset(data_autonomy,Alllow==1 & Allhigh==1)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_all %>%
  group_by(Complexity,Control) %>%
  shapiro_test(RR)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_all,RR,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,RR,
                  wid = .(Subject),
                  within = .(Control,Complexity),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_all$RR,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
data_all$combo <- paste(data_all$Control,data_all$Complexity)
posthoc<-pairwise.t.test(data_all$RR,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

data_high = subset(data_autonomy, Complexity=='high' & Allhigh==1)
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(RR)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_high,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_high$RR,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("########################## LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
data_low = subset(data_autonomy,Complexity=='low' & Alllow==1)
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(RR)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_low,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,RR,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)
posthoc<-pairwise.t.test(data_low$RR,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
