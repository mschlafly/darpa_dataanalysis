################################################################################
# This program performs ANOVA repeated measures statistical tests for darpa HST data.
# In particular, it is used to perform statistical tests on the I*nput, R*R_mean, and N*_obs 
# outcome measures. To test a different metric, ctl find replace the variable 
# (without the asterisk) and change the metric variable
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
#   1. aov(Mean~(SupportLevel*SL_setnum) + Error(wid = .(Subject)/(SupportLevel*SL_setnum)))
#   2. ezANOVA(data,dv=Mean,wid=wid = .(Subject),within = .(SupportLevel,SL_setnum),between = NULL, type = 2, detailed = TRUE)
# The second also tests for sphericity, but sometimes gives eRR_meanors if nothing is significant.
################################################################################

options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())

# loads packages and dependencies
library(ez)
library(rstatix) # for the %>% function


library(lme4) # lmer
# library(nlme) #lme


# parameters
DIR = '/home/milli/Desktop/darpa_dataanalysis/src'
skill = "all" # either "expert", "novice", or "all"
metric = "R*R_mean" # either "I*nput", "R*R_mean", or "N*_obs"

################################################################################
################################################################################
#               Import Data
################################################################################
################################################################################
data_original = read.csv(paste(DIR,"raw_data_formatted","raw_data_formatted.csv",sep="/"))
data_original = subset(data_original, Include_RR_mean=='True')
data_original = subset(data_original, RR_mean!='NaN')
if (skill=="expert"){
  data_all = subset(data_original, Lifetime>=1000)
  # data_control = subset(data_original, Expertise=='expert')
} else if (skill=="novice"){
  data_all = subset(data_original, Lifetime<1000)
  # data_control = subset(data_original, Expertise=='novice')
} else {
  data_all = data_original
}
if (metric=="I*nput"){
  data_all = subset(data_all, Control!='none' & Control!='autoergodic')
}

# define file to save data to
sink(paste(DIR,"Stats","RR_mean",paste("RR_mean",skill,"rm.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Density=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Check assumptions
# Assumption 1: Independence of samples - satisfied by the experimental set-up
# Assumption 2: Data is normally distributed - check visually using boxplots or histograms
#               AND use the shapiro test (if the null hypothesis is violated p<.05, it is not
#               normally distributed
cat("\n")
cat("################################################################################ \n")
cat("Test for normality: Shapiro test\n")
cat("The data is generally normally distributed\n")
cat("################################################################################ \n")
normality = data_all %>%
  group_by(Density,Control) %>%
  shapiro_test(RR_mean)
print(normality)


cat("\n")
cat("################################################################################ \n")
cat("ANOVA \n")
cat("################################################################################ \n")
cat("ANOVA test evaluating the degree to which variance in the outcome measure can be\n")
cat("attributed to the experimental group (e.g., control paradigm, environment, expertise. \n")
cat("We use the exANOVA R package.\n")
cat("Significance indicates that performance under at least one the experimental conditions\n")
cat("is different from the rest and that \n")
cat("[insert independent variable driving experimental grouping] affects [insert dependent variable / outcome measure].\n")
cat("The exANOVA also tests for sphericity (one of the assumptions of an ANOVA test) \n")
cat("If the sphericity condition is not met (p<0.05), apply the greenhouse geisser coRR_meanection p[GG]. \n")
cat("################################################################################ \n")

# Assumption 3: Sphericity/Equal variances between treatments - ezANOVA performs this test
#               if the null hypothesis is violated p<.05 for any particular factor, use a coRR_meanected
#               p-value, possibly the Greenhouse-Geisser p[GG]

if (skill=="all") {
  mod.ez<-ezANOVA(data_all,RR_mean,
                  wid = .(Subject),
                  within = .(Control,Density),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_all,RR_mean,
                  wid = .(Subject),
                  within = .(Control,Density),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("After performing an ANOVA, this test helps us determine which of the experimental \n")
cat("group/s are different from the rest.\n")
cat("Since running multiple tests increases the likelihood of getting significant results,\n")
cat("coRR_meanections are applied.\n")
cat("################################################################################ \n")

# If a factor is significant, that means that at least one of the groups is different from the others
# But you still do not know which group is different. T-tests can help.
data_none = subset(data_all, Control=='none')
data_wp = subset(data_all, Control=='waypoint')
data_user = subset(data_all, Control=='directergodic')
data_shared = subset(data_all, Control=='sharedergodic')
data_auto = subset(data_all, Control=='autoergodic')

if (metric=="I*nput"){
  posthoc<-t.test(data_wp$RR_mean,data_user$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*3)
  posthoc<-t.test(data_wp$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*3)
  posthoc<-t.test(data_user$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*3)
} else if (metric=="N*_obs") {
  posthoc<-t.test(data_wp$RR_mean,data_user$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*6)
  posthoc<-t.test(data_wp$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*6)
  posthoc<-t.test(data_wp$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*6)
  posthoc<-t.test(data_user$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*6)
  posthoc<-t.test(data_user$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*6)
  posthoc<-t.test(data_shared$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*6)
} else {
  posthoc<-t.test(data_none$RR_mean,data_wp$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_none$RR_mean,data_user$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_none$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_none$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_wp$RR_mean,data_user$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_wp$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_wp$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_user$RR_mean,data_shared$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_user$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
  posthoc<-t.test(data_shared$RR_mean,data_auto$RR_mean,
                  paired = TRUE,detailed = TRUE)
  print(posthoc)
  print("Adjusted p-val bonferroni")
  print(posthoc$p.value*10)
}
posthoc<-pairwise.t.test(data_all$RR_mean,
                         data_all$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni",detailed = TRUE)
print(posthoc)

# Compare all group/factor combinations for putting asterisks on plots
data_all$combo <- paste(data_all$Control,data_all$Density)
posthoc<-pairwise.t.test(data_all$RR_mean,
                         data_all$combo,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

# If there is an interaction effect between two factors (in this case Density and control),
# analyze trends within each group seperately

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

data_high = subset(data_all, Density=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_high %>%
  group_by(Control) %>%
  shapiro_test(RR_mean)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_high,RR_mean,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_high,RR_mean,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_high$RR_mean,
                         data_high$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("########################## LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
data_low = subset(data_all,Density=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

cat("Test for normality \n")
normality = data_low %>%
  group_by(Control) %>%
  shapiro_test(RR_mean)
print(normality)

if (skill=="all") {
  mod.ez<-ezANOVA(data_low,RR_mean,
                  wid = .(Subject),
                  within = .(Control),
                  between = Expertise, type = 3, detailed = TRUE)
} else {
  mod.ez<-ezANOVA(data_low,RR_mean,
                  wid = .(Subject),
                  within = .(Control),
                  between = NULL, type = 2, detailed = TRUE)
}
print(mod.ez)

posthoc<-pairwise.t.test(data_low$RR_mean,
                         data_low$Control,
                         paired = TRUE,
                         p.adjust.method = "bonferroni")
print(posthoc)
