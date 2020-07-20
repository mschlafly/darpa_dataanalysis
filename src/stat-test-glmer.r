################################################################################
# This program performs statistical tests for darpa HST overall performance data.
# Statistical results are published to output file created by sink()

################################################################################
# INSTRUCTIONS:
#     Change DIR, data, and the sink file before running in the terminal
#     To run in the terminal
#     $ R
#     Then run the script by executing the following
#     > source('stat-test-overall.r')
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
DIR = 'C:/Users/numur/Desktop/darpa_dataanalysis/src' 
skill = "expert" # either "expert", "novice", "compareexpertise", or "all"
                 # for "compareexpertise", test a model using just expertise and control
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
data_control_original = read.csv(paste(DIR,paste("control",".csv",sep=""),sep="/"))
if (skill=="expert"){
  data_control = subset(data_control_original, Lifetime>999)
} else if (skill=="novice"){
  data_control = subset(data_control_original, Lifetime<999)
} else {
  data_control = data_control_original
}
data_method_original = read.csv(paste(DIR,paste("method","average.csv",sep="_"),sep="/"))
if (skill=="expert"){
  data_method = subset(data_method_original, Lifetime>999)
} else if (skill=="novice"){
  data_method = subset(data_method_original, Lifetime<999)
} else {
  data_method = data_method_original
}
data_method_direct = subset(data_control, Control!='sharedergodic' & Control!='autoergodic')
data_method_shared = subset(data_control, Control!='directergodic' & Control!='autoergodic')
data_method_auto = subset(data_control, Control!='directergodic' & Control!='sharedergodic')
data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')

################################################################################
################################################################################
#               CONTROL*
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("control","Lives",skill,"2.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_control,Uselow==1 & Usehigh==1)
} else {
  data_all = data_control
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Lives = (-data_all$Lives)+max(data_all$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
# I tried with all participants - L*ives sos error - S*core sos error
#     Gamma*(link = "identity") - 
#     Gamma*(link = "log") - 
#     Gamma*(link = "inverse") - 
#     poisson*(link = "log") - 
#     poisson*(link = "identity") - 
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~ Control * Expertise + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Complexity * Control + (1|Subject),
                      data = data_all, 
                      family = poisson(link = "identity"),
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

# Calculate anova
anov = Anova(glmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(glmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))

# Alternate methods of doing a post-hoc tests
if (skill!="compareexpertise") {
  library(emmeans)
  marginal = lsmeans(glmer_model,~ Complexity * Control)
  print(pairs(marginal,adjust="tukey"))  
}

# # Post-hoc t-test with bonferroni correction - assumes normality
# data_all$combo <- paste(data_all$Control,data_all$Complexity)
# posthoc<-pairwise.t.test(data_all$Lives,data_all$combo,paired = TRUE, p.adjust.method = "bonferroni")
# print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_high = subset(data_control,Complexity=='high' & Usehigh==1)
} else {
  data_high = subset(data_control,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Lives = (-data_high$Lives)+max(data_high$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_high, 
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_high, 
                      family = poisson(link = "identity"),
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
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_low = subset(data_control,Complexity=='low' & Usehigh==1)
} else {
  data_low = subset(data_control,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Lives = (-data_low$Lives)+max(data_low$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_low, 
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_low, 
                      family = poisson(link = "identity"),
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

# ################################################################################
# ################################################################################
# #               METHOD - AVERAGE
# ################################################################################
# ################################################################################
# 
# # define file to save data to
# sink(paste(DIR,"stattests",paste("avgmethod","Lives",skill,"2.txt",sep="-"),sep="/"))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### ALL ################################### \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_all = subset(data_method,Uselow==1 & Usehigh==1)
# } else {
#   data_all = data_method
# }
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform left-skewed data so that it is right-skewed
# data_all$Lives = (-data_all$Lives)+max(data_all$Lives)+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Lives ~  Complexity * Method * Expertise + (1|Subject),
#                       data = data_all, 
#                       family = poisson(link = "identity"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Lives ~  Complexity * Method + (1|Subject),
#                       data = data_all, 
#                       family = poisson(link = "identity"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4))) 
# }
# print("Sum of squares")
# sos = sum(resid(glmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos 
# 
# # plotting to evaluate assumptions
# plot(glmer_model)
# 
# # Calculate anova
# anov = Anova(glmer_model,type="III")
# print(anov)
# 
# # Post-hoc tukey tests
# print(summary(glht(glmer_model,
#                    linfct=mcp(Method = "Tukey",
#                               interaction_average = TRUE))))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### HIGH ################################## \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_high = subset(data_method,Complexity=='high' & Usehigh==1)
# } else {
#   data_high = subset(data_method,Complexity=='high')
# }
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform left-skewed data so that it is right-skewed
# data_high$Lives = (-data_high$Lives)+max(data_high$Lives)+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Lives ~  Method * Expertise + (1|Subject),
#                       data = data_high, 
#                       family = poisson(link = "identity"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Lives ~  Method + (1|Subject),
#                       data = data_high, 
#                       family = poisson(link = "identity"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))  
# }
# print("Sum of squares")
# sos = sum(resid(glmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos 
# 
# # plotting to evaluate assumptions
# plot(glmer_model)
# 
# # Calculate anova
# anov = Anova(glmer_model,type="III")
# print(anov)
# 
# # Post-hoc tukey tests
# print(summary(glht(glmer_model,
#                    linfct=mcp(Method = "Tukey",
#                               interaction_average = TRUE))))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### LOW ################################### \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_low = subset(data_method,Complexity=='low' & Usehigh==1)
# } else {
#   data_low = subset(data_method,Complexity=='low')
# }
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform left-skewed data so that it is right-skewed
# data_low$Lives = (-data_low$Lives)+max(data_low$Lives)+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Lives ~  Method * Expertise + (1|Subject),
#                       data = data_low, 
#                       family = poisson(link = "identity"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Lives ~  Method + (1|Subject),
#                       data = data_low, 
#                       family = poisson(link = "identity"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))  
# }
# print("Sum of squares")
# sos = sum(resid(glmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos 
# 
# # plotting to evaluate assumptions
# plot(glmer_model)
# 
# # Calculate anova
# anov = Anova(glmer_model,type="III")
# print(anov)
# 
# # Post-hoc tukey tests
# print(summary(glht(glmer_model,
#                    linfct=mcp(Method = "Tukey",
#                               interaction_average = TRUE))))

################################################################################
################################################################################
#               METHOD - DIRECT
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("directmethod","Lives",skill,"2.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_method_direct,Uselow==1 & Usehigh==1)
} else {
  data_all = data_method_direct
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Lives = (-data_all$Lives)+max(data_all$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
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
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_high = subset(data_method_direct,Complexity=='high' & Usehigh==1)
} else {
  data_high = subset(data_method_direct,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Lives = (-data_high$Lives)+max(data_high$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
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
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_low = subset(data_method_direct,Complexity=='low' & Usehigh==1)
} else {
  data_low = subset(data_method_direct,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Lives = (-data_low$Lives)+max(data_low$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
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
#               METHOD - SHARED
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("sharedmethod","Lives",skill,"2.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_method_shared,Uselow==1 & Usehigh==1)
} else {
  data_all = data_method_shared
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Lives = (-data_all$Lives)+max(data_all$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
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
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_high = subset(data_method_shared,Complexity=='high' & Usehigh==1)
} else {
  data_high = subset(data_method_shared,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Lives = (-data_high$Lives)+max(data_high$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
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
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_low = subset(data_method_shared,Complexity=='low' & Usehigh==1)
} else {
  data_low = subset(data_method_shared,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Lives = (-data_low$Lives)+max(data_low$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
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
#               METHOD - AUTO
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("automethod","Lives",skill,"2.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_method_auto,Uselow==1 & Usehigh==1)
} else {
  data_all = data_method_auto
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Lives = (-data_all$Lives)+max(data_all$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
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
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_high = subset(data_method_auto,Complexity=='high' & Usehigh==1)
} else {
  data_high = subset(data_method_auto,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Lives = (-data_high$Lives)+max(data_high$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
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
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_low = subset(data_method_auto,Complexity=='low' & Usehigh==1)
} else {
  data_low = subset(data_method_auto,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Lives = (-data_low$Lives)+max(data_low$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
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
sink(paste(DIR,"stattests",paste("autonomy","Lives",skill,"2.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_all = subset(data_autonomy,Uselow==1 & Usehigh==1)
} else {
  data_all = data_autonomy
}
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_all$Lives = (-data_all$Lives)+max(data_all$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      family = poisson(link = "identity"),
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
cat("################################### HIGH ################################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_high = subset(data_autonomy,Complexity=='high' & Usehigh==1)
} else {
  data_high = subset(data_autonomy,Complexity=='high')
}
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_high$Lives = (-data_high$Lives)+max(data_high$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_high,
                      family = poisson(link = "identity"),
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
cat("################################### LOW ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
if (onlySubset=="T") {
  data_low = subset(data_autonomy,Complexity=='low' & Usehigh==1)
} else {
  data_low = subset(data_autonomy,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Lives = (-data_low$Lives)+max(data_low$Lives)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Lives ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Lives ~  Control + (1|Subject),
                      data = data_low,
                      family = poisson(link = "identity"),
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
