################################################################################
# This program performs statistical tests for darpa HST overall performance data.
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
# DIR = 'C:/Users/milli/OneDrive/Documents/darpa_dataanalysis/src'
DIR = '/home/milli/Desktop/darpa_dataanalysis/src'
skill = "expert" # either "expert", "novice", "compareexpertise", or "all"
                 # (but "all" has convergence problems)
                 # instead, use "compareexpertise", test a model using just expertise and control
onlySubset = "F" # "T" for true and "F" for false
                 # use a type II anova for T and III for false
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
data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs_i_6a_10000s.csv",sep="/"))
# data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs_cum_6a_10000s.csv",sep="/"))

# data_original = subset(data_original, Include_P_switch=='True')
# # data_original = subset(data_original, P_switch!='NaN')
# if (skill=="expert"){
#   data_control = subset(data_original, Lifetime>999)
# } else if (skill=="novice"){
#   data_control = subset(data_original, Lifetime<999)
# } else {
#   data_control = data_original
# }
data_control = data_original
data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')

################################################################################
################################################################################
#               CONTROL*
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","P_switch",paste("control","P_switch",skill,"glmer.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Complexity=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis

data_all = data_control
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch ~ Control * Expertise + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch ~  Complexity * Control + (1|Subject),
                      data = data_all,
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
# qqnorm(resid(glmer_model))
# hist(resid(glmer_model))
# library(sjPlot)
# plot_model(glmer_model, type = "pred", show.data = TRUE)

# print(summary(glmer_model))

# Calculate anova
anov = Anova(glmer_model,type="III",
             test.statistic = "Chisq")
print(anov)

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
data_high = subset(data_control,Complexity=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch ~  Control + (1|Subject),
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
data_low = subset(data_control,Complexity=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch ~  Control + (1|Subject),
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

################################################################################
################################################################################
#               AUTONOMY
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","P_switch",paste("autonomy","P_switch",skill,"glmer.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis

data_all = data_autonomy
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch ~  Complexity * Control + (1|Subject),
                      data = data_all,
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

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis

data_high = subset(data_autonomy,Complexity=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch ~  Control + (1|Subject),
                      data = data_high,
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

cat("\n")
cat("########################################################################### \n")
cat("######################### LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_low = subset(data_autonomy,Complexity=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(P_switch ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(P_switch ~  Control + (1|Subject),
                      data = data_low,
                      family = binomial(link = "logit"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
}
print("Sum of squares")
sos = sum(resid(glmer_model)^2)
print(sos)
modelerrorsum = modelerrorsum + sos
print("Sum of all model error")
print(modelerrorsum)

# plotting to evaluate assumptions
plot(glmer_model)

# Calculate anova
anov = Anova(glmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(glmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))
