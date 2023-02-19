################################################################################
# This program performs statistical tests for overall performance data ("S*core")and 
# cumulative regret with all environment information ("M*DP_r")
# To test a different metric, find and replace all instances of the metric name 
# above without the asterisk.
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
skill = "novice" # either "expert", "novice", "compareexpertise", or "all"
                 # (but "all" has convergence problems)
                 # instead, use "compareexpertise", test a model using just expertise and control
metric = "M*DP_r" # either "M*DP_r" or S*core ; make sure to find and replace all instances in this code as well
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
data_original = subset(data_original, Include_MDP_r=='True')
data_original = subset(data_original, MDP_r!='NaN')
if (skill=="expert"){
  data_control = subset(data_original, Lifetime>=1000)
} else if (skill=="novice"){
  data_control = subset(data_original, Lifetime<1000)
} else {
  data_control = data_original
}
print(data_control)

################################################################################
################################################################################
#               CONTROL*
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","MDP_r",paste("MDP_r",skill,"glmer.txt",sep="-"),sep="/"))

# This plot shows that the input data is left-skewed
# library(rcompanion)
# plotNormalHistogram(data_control$MDP_r)

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Density=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")
data_all = data_control
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
if (metric!="M*DP_r"){
  data_all$MDP_r = (-data_all$MDP_r)+max(data_all$MDP_r)+1
}

# Build generalized linear mixed-effects model (GLMM)
# I tried with all participants - L*ives sos error - S*core sos error
#     Gamma*(link = "identity") -
#     Gamma*(link = "log") -
#     Gamma*(link = "inverse") -
#     poisson*(link = "log") -
#     poisson*(link = "identity") -
if (skill=="compareexpertise") {
  glmer_model = glmer(MDP_r ~ Control * Expertise + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(MDP_r ~  Density * Control + (1|Subject),
                      data = data_all,
                      poisson(link = "identity"),
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

# # Alternate methods of doing a post-hoc tests
# if (skill!="compareexpertise") {
#   library(emmeans)
#   marginal = lsmeans(glmer_model,~ Density * Control)
#   print(pairs(marginal,adjust="tukey"))
# }

# # Post-hoc t-test with bonferroni correction - assumes normality
# data_all$combo <- paste(data_all$Control,data_all$Density)
#
# print(data_all$combo)
# posthoc<-pairwise.t.test(data_all$MDP_r,data_all$combo,paired = TRUE, p.adjust.method = "bonferroni")
# print(posthoc)

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_high = subset(data_control,Density=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
if (metric!="M*DP_r"){
  data_high$MDP_r = (-data_high$MDP_r)+max(data_high$MDP_r)+1
}
# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(MDP_r ~  Control * Expertise + (1|Subject),
                      data = data_high,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(MDP_r ~  Control + (1|Subject),
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
data_low = subset(data_control,Density=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

# Transform left-skewed data so that it is right-skewed
if (metric!="M*DP_r"){
  data_low$MDP_r = (-data_low$MDP_r)+max(data_low$MDP_r)+1
}
# Build generalized linear mixed-effects model (GLMM)
if (skill=="compareexpertise") {
  glmer_model = glmer(MDP_r ~  Control * Expertise + (1|Subject),
                      data = data_low,
                      poisson(link = "identity"),
                      control=glmerControl(check.conv.singular = .makeCC(action = "ignore",  tol = 1e-4)))
} else {
  glmer_model = glmer(MDP_r ~  Control + (1|Subject),
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
