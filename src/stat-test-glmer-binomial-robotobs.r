################################################################################
# This program performs statistical tests for darpa HST overall performance data.
# In particular, it is used to perform statistical tests on the P*_switch and P*_switch_cum
# outcome measures. To test a different metric, ctl find replace the variable 
# (without the asterisk) and change the metric variable
# Statistical results are published to output file created by sink()

################################################################################
# INSTRUCTIONS:
#     Change DIR, data, and the sink file before running in the terminal
#     To run in the terminal
#     $ R
#     Then run the script by executing the following
#     > source('stat-test-glmer.r')
#     If running from console in RStudio on windows computer, use setwd
#     for example - setwd('C:/Users/numur/Desktop/darpa_dataanalysis/src')
################################################################################
# COMMENTS:
# - To test a different metric, Ctrl replace the name of the metric


################################################################################
# useful link https://www.youtube.com/watch?v=wWjVQ8eqR9k
options(contrasts=c("contr.sum","contr.poly"))
remove(list = ls())

# parameters
DIR = '/home/milli/Desktop/darpa_dataanalysis/src'
skill = "novice" # either "expert", "novice", "compareexpertise", or "all"
                 # instead, use "compareexpertise", test a model using just expertise and control
onlySubset = "F" # "T" for true and "F" for false
                 # use a type II anova for T and III for false
metric = "P*_switch_cum" # either "P*_switch" or "P*_switch_cum" ; make sure to find and replace all instances in this code as well
modelerrorsum = 0

# loads packages and dependencies
library(ez)
library(car)
library(lme4)
library(multcomp)

################################################################################
################################################################################
#               Import Data
################################################################################
################################################################################
if (metric=="P*_switch"){
  data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs_i_6a_10000s.csv",sep="/"))
}
if (metric=="P*_switch_cum"){
  data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs_cum_6a_10000s.csv",sep="/"))
}

# data_original = subset(data_original, Include_P_switch_cum=='True')
data_original = subset(data_original, P_switch_cum!='NaN')
if (skill=="expert"){
  data_control = subset(data_original, Expertise=="expert")
} else if (skill=="novice"){
  data_control = subset(data_original, Expertise=="novice")
} else {
  data_control = data_original
}

# define file to save data to
sink(paste(DIR,"Stats","P_switch_cum",paste("P_switch_cum",skill,"glmer.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Density=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis

data_all = data_control
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# library(coin)
# test = independence_test(P_switch_cum ~ Control,
#                   data = data_all)
# print(summary(test))
# print(test)

# cat("Test for normality \n")
# # print(data_all %>%
# #         group_by(Density,Control))
# library(rstatix) # for the %>% function
# normality = data_all %>%
#   group_by(Density,Control) %>%
#   shapiro_test(P_switch_cum)
# print(normality)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch_cum ~ Control * Expertise + (1|Subject),
                      data = data_all,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch_cum ~ Density * Control + (1|Subject),
                      data = data_all,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
}
# print("Sum of squares")
sos = sum(resid(glmer_model)^2)
# print(sos)
modelerrorsum = modelerrorsum + sos
# print(summary(glmer_model))

# plotting to evaluate assumptions
plot(glmer_model)
# qqnorm(resid(glmer_model))
# hist(resid(glmer_model))
# library(sjPlot)
# plot_model(glmer_model, type = "pred", show.data = TRUE)

# print(summary(glmer_model))

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
cat("If the sphericity condition is not met (p<0.05), apply the greenhouse geisser correction p[GG]. \n")
cat("################################################################################ \n")
# Calculate anova
anov = Anova(glmer_model,type="III",
             test.statistic = "Chisq")
print(anov)

cat("################################################################################ \n")
cat("T-test for group difference \n")
cat("After performing an ANOVA, this test helps us determine which of the experimental \n")
cat("group/s are different from the rest.\n")
cat("Since running multiple tests increases the likelihood of getting significant results,\n")
cat("corrections are applied.\n")
cat("################################################################################ \n")
# Post-hoc tukey
dof = nrow(data_all)-5
print("degrees of freedom:")
print(dof)
print(summary(glht(glmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE),df=dof)))


cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_high = subset(data_control,Density=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch_cum ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch_cum ~  Control + (1|Subject),
                      data = data_high,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
}
print("Sum of squares")
sos = sum(resid(glmer_model)^2)
print(sos)
modelerrorsum = modelerrorsum + sos
# print(summary(glmer_model))

# plotting to evaluate assumptions
plot(glmer_model)

# Calculate anova
anov = Anova(glmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(glmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))

cat("\n")
cat("########################################################################### \n")
cat("########################## LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")


# Select subset of the data for analysis
data_low = subset(data_control,Density=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)


# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch_cum ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch_cum ~  Control + (1|Subject),
                      data = data_low,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
}
print("Sum of squares")
sos = sum(resid(glmer_model)^2)
print(sos)
modelerrorsum = modelerrorsum + sos

# plotting to evaluate assumptions
plot(glmer_model)

# Calculate anova
anov = Anova(glmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(glmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))
