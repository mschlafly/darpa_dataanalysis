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
DIR = 'C:/Users/numur/Desktop/darpa_data_analysis/src' 
skill = "compareexpertise" # either "expert", "novice", "compareexpertise", or "all"
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

# ################################################################################
# ################################################################################
# #                                   Inputs                                     #
# ################################################################################
# ################################################################################
# 
# # define file to save data to
# sink(paste(DIR,"stattests",paste("inputs","Input",skill,"glmer.txt",sep="-"),sep="/"))
# 
# # This plot shows that the input data is right-skewed
# #     -> no transformation is required to fit a polynomial distribution
# #     -> since '0' is a possible measure, this code adds 1 to all points in 
# #         the distribution
# # library(rcompanion)
# # plotNormalHistogram(data_inputs$Input)
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### ALL ################################### \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_all = subset(data_inputs,Uselow==1 & Usehigh==1)
# } else {
#   data_all = data_inputs
# }
# data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform data so that there is no zeros
# data_all$Input = data_all$Input+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# # I tried with all participants - L*ives sos error - S*core sos error
# #     Gamma*(link = "identity") -
# #     Gamma*(link = "inverse") -
# #     Gamma*(link = "inverse") -
# #     poisson*(link = "inverse") -
# #     poisson*(link = "identity") -
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Input ~ Control * Expertise + (1|Subject),
#                       data = data_all,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Input ~  Complexity * Control + (1|Subject),
#                       data = data_all,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# }
# print("Sum of squares")
# sos = sum(resid(glmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# # print(summary(glmer_model))
# 
# # plotting to evaluate assumptions
# plot(glmer_model)
# # qqnorm(resid(glmer_model))
# # hist(resid(glmer_model))
# # library(sjPlot)
# # plot_model(glmer_model, type = "pred", show.data = TRUE)
# 
# # Calculate anova
# anov = Anova(glmer_model,type="III")
# print(anov)
# 
# # Post-hoc tukey tests
# print(summary(glht(glmer_model,
#                    linfct=mcp(Control = "Tukey",
#                               interaction_average = TRUE))))
# 
# # Alternate methods of doing a post-hoc tests
# if (skill!="compareexpertise") {
#   library(emmeans)
#   marginal = lsmeans(glmer_model,~ Complexity * Control)
#   print(pairs(marginal,adjust="tukey"))
# }
# 
# # # Post-hoc t-test with bonferroni correction - assumes normality
# # data_all$combo <- paste(data_all$Control,data_all$Complexity)
# # posthoc<-pairwise.t.test(data_all$Difficulty,data_all$combo,paired = TRUE, p.adjust.method = "bonferroni")
# # print(posthoc)
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### HIGH ################################## \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_high = subset(data_control,Complexity=='high' & Usehigh==1)
# } else {
#   data_high = subset(data_control,Complexity=='high')
# }
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform data so that there is no zeros
# data_high$Input = data_high$Input+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Input ~  Control * Expertise + (1|Subject),
#                       data = data_high,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Input ~  Control + (1|Subject),
#                       data = data_high,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# }
# print("Sum of squares")
# sos = sum(resid(glmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# # print(summary(glmer_model))
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
#                    linfct=mcp(Control = "Tukey",
#                               interaction_average = TRUE))))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### LOW ################################### \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_low = subset(data_control,Complexity=='low' & Uselow==1)
# } else {
#   data_low = subset(data_control,Complexity=='low')
# }
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform data so that there is no zeros
# data_low$Input = data_low$Input+1
# 
# print("Skipped because the model failed and complexity*control is not a significant interaction effect")
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   # glmer_model = glmer(Input ~  Control * Expertise + (1|Subject),
#   #                     data = data_low,
#   #                     family = Gamma(link = "log"),
#   #                     control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   # glmer_model = glmer(Input ~  Control + (1|Subject),
#   #                     data = data_low,
#   #                     family = Gamma(link = "log"),
#   #                     control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# }
# # print("Sum of squares")
# # sos = sum(resid(glmer_model)^2)
# # print(sos)
# # modelerrorsum = modelerrorsum + sos
# # 
# # # plotting to evaluate assumptions
# # plot(glmer_model)
# # 
# # # Calculate anova
# # anov = Anova(glmer_model,type="III")
# # print(anov)
# # 
# # # Post-hoc tukey tests
# # print(summary(glht(glmer_model,
# #                    linfct=mcp(Control = "Tukey",
# #                               interaction_average = TRUE))))

################################################################################
################################################################################
#               CONTROL*
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"stattests",paste("control","Difficulty",skill,"glmer.txt",sep="-"),sep="/"))

# This plot shows that the input data is left-skewed
# library(rcompanion)
# plotNormalHistogram(data_control$Difficulty)

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
data_all$Difficulty = (-data_all$Difficulty)+max(data_all$Difficulty)+1

# Build generalized linear mixed-effects model (GLMM)
# I tried with all participants - L*ives sos error - S*core sos error
#     Gamma*(link = "identity") - 57.61263
#     Gamma*(link = "log") - 55.7552
#     Gamma*(link = "inverse") - 58.89034
#     poisson*(link = "inverse") - 161.846
#     poisson*(link = "identity") - 196.4355
if (skill=="compareexpertise") {
  glmer_model = glmer(Difficulty ~ Control * Expertise + (1|Subject),
                      data = data_all,
                      family = Gamma(link = "log"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Difficulty ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      family = Gamma(link = "log"),
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
# posthoc<-pairwise.t.test(data_all$Difficulty,data_all$combo,paired = TRUE, p.adjust.method = "bonferroni")
# print(posthoc)
# 
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
data_high$Difficulty = (-data_high$Difficulty)+max(data_high$Difficulty)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Difficulty ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      family = Gamma(link = "log"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Difficulty ~  Control + (1|Subject),
                      data = data_high,
                      family = Gamma(link = "log"),
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
  data_low = subset(data_control,Complexity=='low' & Uselow==1)
} else {
  data_low = subset(data_control,Complexity=='low')
}
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
data_low$Difficulty = (-data_low$Difficulty)+max(data_low$Difficulty)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Difficulty ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      family = Gamma(link = "log"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Difficulty ~  Control + (1|Subject),
                      data = data_low,
                      family = Gamma(link = "log"),
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
sink(paste(DIR,"stattests",paste("autonomy","Difficulty",skill,"glmer.txt",sep="-"),sep="/"))

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
data_all$Difficulty = (-data_all$Difficulty)+max(data_all$Difficulty)+1

# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(Difficulty ~  Control * Expertise + (1|Subject),
                      data = data_all,
                      family = Gamma(link = "log"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(Difficulty ~  Complexity * Control + (1|Subject),
                      data = data_all,
                      family = Gamma(link = "log"),
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

# cat("\n")
# cat("########################################################################### \n")
# cat("################################### HIGH ################################## \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_high = subset(data_autonomy,Complexity=='high' & Usehigh==1)
# } else {
#   data_high = subset(data_autonomy,Complexity=='high')
# }
# data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform left-skewed data so that it is right-skewed
# data_high$Difficulty = (-data_high$Difficulty)+max(data_high$Difficulty)+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Difficulty ~  Control * Expertise + (1|Subject),
#                       data = data_high,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Difficulty ~  Control + (1|Subject),
#                       data = data_high,
#                       family = Gamma(link = "log"),
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
#                    linfct=mcp(Control = "Tukey",
#                               interaction_average = TRUE))))
# 
# cat("\n")
# cat("########################################################################### \n")
# cat("################################### LOW ################################### \n")
# cat("########################################################################### \n")
# 
# # Select subset of the data for analysis
# if (onlySubset=="T") {
#   data_low = subset(data_autonomy,Complexity=='low' & Uselow==1)
# } else {
#   data_low = subset(data_autonomy,Complexity=='low')
# }
# data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)
# 
# # Transform left-skewed data so that it is right-skewed
# data_low$Difficulty = (-data_low$Difficulty)+max(data_low$Difficulty)+1
# 
# # Build generalized linear mixed-effects model (GLMM)
# if (skill=="compareexpertise") {
#   glmer_model = glmer(Difficulty ~  Control * Expertise + (1|Subject),
#                       data = data_low,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# } else {
#   glmer_model = glmer(Difficulty ~  Control + (1|Subject),
#                       data = data_low,
#                       family = Gamma(link = "log"),
#                       control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
# }
# print("Sum of squares")
# sos = sum(resid(glmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# print("Sum of all model error")
# print(modelerrorsum)
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
#                    linfct=mcp(Control = "Tukey",
#                               interaction_average = TRUE))))
