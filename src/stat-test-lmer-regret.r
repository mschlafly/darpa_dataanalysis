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
DIR = '/home/milli/Desktop/darpa_dataanalysis/src'
skill = "expert" # either "expert", "novice", "compareexpertise", or "all"
# instead, use "compareexpertise", test a model using just expertise and control
# modelerrorsum = 0

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
data_original = read.csv(paste(DIR,"raw_data_formatted","raw_data_formatted_POMDP_d.csv",sep="/"))
# data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_d_6a_10000s.csv",sep="/"))
if (skill=="expert"){
  data_all = subset(data_original, Expertise=='expert')
} else if (skill=="novice"){
  data_all = subset(data_original, Expertise=='novice')
} else {
  data_all = data_original
}
# data_control = data_original

# define file to save data to
sink(paste(DIR,"Stats","Regret",paste("Regret",skill,"lmer.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Density=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

if (skill=="novice"){
  cat("\n\n\n\n\n SINGULAR FIT \n\n\n\n\n")
}
  
  
# Select subset of the data for analysis
# data_all = data_control
data_all = subset(data_all,Maximum_Regret<3)
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

# Separate Maximum_Regret into 3 max_regret_groups of equal range
categories <- cut(data_all$Maximum_Regret,3)
cat("Here are the Maximum_Regret group ranges: ")
print(levels(categories))
df <- data.frame(data_all$Maximum_Regret,categories)
data_all$max_regret_groups <- df$categories

boxplot(Percent_regret ~ Control, data = data_all)

if (skill=="compareexpertise") {
    lmer_model = lmer(Regret ~  Control * Expertise * max_regret_groups + (1|Subject),
                      data = data_all)
} else {
    lmer_model = lmer(Regret ~  Control * Density * max_regret_groups + (1|Subject),
                      data = data_all)
}
# print("Sum of squares")
# sos = sum(resid(lmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# print(summary(lmer_model))
# # plotting to evaluate assumptions
# plot(lmer_model)
# qqnorm(resid(lmer_model))
# hist(resid(lmer_model))
# library(sjPlot)
# plot_model(lmer_model, type = "pred", show.data = TRUE)


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
anov = Anova(lmer_model,type="III")
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
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              covariate_average = TRUE),df=dof)))

if (skill!="compareexpertise") {
# if (skill!="novice") {
cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_high = subset(data_all,Density=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

if (skill=="compareexpertise") {
  lmer_model = lmer(Regret ~ Control * Expertise * max_regret_groups + (1|Subject),
                    data = data_high)
} else {
  lmer_model = lmer(Regret ~ Control * max_regret_groups + (1|Subject),
                    data = data_high)
}
# print("Sum of squares")
# sos = sum(resid(lmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# print(summary(lmer_model))
# plotting to evaluate assumptions
# plot(lmer_model)

# Calculate anova
anov = Anova(lmer_model,type="III")
print(anov)

# Post-hoc tukey tests
dof = nrow(data_high)-5
print("degrees of freedom:")
print(dof)
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              covariate_average = TRUE))))

cat("\n")
cat("########################################################################### \n")
cat("########################## LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_low = subset(data_all,Density=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

if (skill=="compareexpertise") {
    lmer_model = lmer(Regret ~ Control * Expertise * max_regret_groups + (1|Subject),
                      data = data_low)
} else {
    lmer_model = lmer(Regret ~ Control * max_regret_groups + (1|Subject),
                      data = data_low)
}
# print("Sum of squares")
# sos = sum(resid(lmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# # plotting to evaluate assumptions
# plot(lmer_model)

# Calculate anova
anov = Anova(lmer_model,type="III")
print(anov)

# Post-hoc tukey tests
dof = nrow(data_low)-5
print("degrees of freedom:")
print(dof)
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              covariate_average = TRUE))))

cat("\n")
cat("########################################################################### \n")
cat("####################### SUBmax_regret_groups OF MAXIMUM_REGRET  ###################### \n")
cat("###### where maximum regret is the impact fo the decision on reward ####### \n")
cat("########################################################################### \n")

for(i in 1:length(levels(data_all$max_regret_groups))){
  cat("\n For level ")
  print(levels(data_all$max_regret_groups)[i])
  data_subset = subset(data_all,max_regret_groups==levels(data_all$max_regret_groups)[i])
  data_subset[] <- lapply(data_subset, function(x) if(is.factor(x)) factor(x) else x)
  lmer_model = lmer(Regret ~  Density * Control + (1|Subject),
                    data = data_subset)
  anov = Anova(lmer_model,type="III")
  print(anov)
  dof = nrow(data_subset)-5
  print("degrees of freedom:")
  print(dof)
  print(summary(glht(lmer_model,
                     linfct=mcp(Control = "Tukey",
                                covariate_average = TRUE),df=dof)))
}
}