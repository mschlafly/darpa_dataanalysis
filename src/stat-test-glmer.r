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
skill = "compareexpertise" # either "expert", "novice", "compareexpertise", or "all"
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
data_original = read.csv(paste(DIR,"raw_data_formatted","raw_data_formatted.csv",sep="/"))
data_original = subset(data_original, Include_Score=='True')
data_original = subset(data_original, Score!='NaN')
if (skill=="expert"){
  data_control = subset(data_original, Lifetime>999)
} else if (skill=="novice"){
  data_control = subset(data_original, Lifetime<999)
} else {
  data_control = data_original
}
data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')

################################################################################
################################################################################
#               CONTROL*
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","Score",paste("control","Score",skill,"glmer.txt",sep="-"),sep="/"))

# This plot shows that the input data is left-skewed
# library(rcompanion)
# plotNormalHistogram(data_control$Score)

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Complexity=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_control,Alllow==1 & Allhigh==1)
} else {
  data_all = data_control
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Score = (-data_all$Score)+max(data_all$Score)+1

# Build generalized linear mixed-effects model (GLMM)
# I tried with all participants - L*ives sos error - S*core sos error
#     Gamma*(link = "identity") -
#     Gamma*(link = "log") -
#     Gamma*(link = "inverse") -
#     poisson*(link = "log") -
#     poisson*(link = "identity") -
if (skill=="compareexpertise") {
  glmer_model = glmer(Score ~ Control * Expertise + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Score ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
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

# # Alternate methods of doing a post-hoc tests
# if (skill!="compareexpertise") {
#   library(emmeans)
#   marginal = lsmeans(glmer_model,~ Complexity * Control)
#   print(pairs(marginal,adjust="tukey"))
# }

# # Post-hoc t-test with bonferroni correction - assumes normality
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
#
# print(data_all$combo)
# posthoc<-pairwise.t.test(data_all$Score,data_all$combo,paired = TRUE, p.adjust.method = "bonferroni")
# print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_high = subset(data_control,Complexity=='high' & Allhigh==1)
} else {
  data_high = subset(data_control,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Score = (-data_high$Score)+max(data_high$Score)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Score ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Score ~  Control + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
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
if (onlySubset=="T") {
  data_low = subset(data_control,Complexity=='low' & Alllow==1)
} else {
  data_low = subset(data_control,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Score = (-data_low$Score)+max(data_low$Score)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Score ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Score ~  Control + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
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
sink(paste(DIR,"Stats","Score",paste("autonomy","Score",skill,"glmer.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_autonomy,Alllow==1 & Allhigh==1)
} else {
  data_all = data_autonomy
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Score = (-data_all$Score)+max(data_all$Score)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Score ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Score ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
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
if (onlySubset=="T") {
  data_high = subset(data_autonomy,Complexity=='high' & Allhigh==1)
} else {
  data_high = subset(data_autonomy,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Score = (-data_high$Score)+max(data_high$Score)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Score ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Score ~  Control + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
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
if (onlySubset=="T") {
  data_low = subset(data_autonomy,Complexity=='low' & Alllow==1)
} else {
  data_low = subset(data_autonomy,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Score = (-data_low$Score)+max(data_low$Score)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Score ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Score ~  Control + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
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
