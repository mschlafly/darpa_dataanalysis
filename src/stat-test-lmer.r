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
# data_original = read.csv(paste(DIR,"raw_data_formatted","raw_data_formatted.csv",sep="/"))
data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_d_6a_10000s.csv",sep="/"))
# data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs_i_6a_10000s.csv",sep="/"))
# data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs_cum_6a_10000s.csv",sep="/"))
# data_original = read.csv(paste(DIR,"raw_data","raw_data_POMDP_obs.csv",sep="/"))

data_control = data_original
data_autonomy = subset(data_control,Control!='none' & Control!='waypoint')

################################################################################
################################################################################
#               CONTROL*
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","Regret",paste("control","Regret",skill,"lmer.txt",sep="-"),sep="/"))

cat("\n")
cat("################################################################################ \n")
cat("###############               All Experimental Factors            ############## \n")
cat("####### Complexity=(low/high buiding density) Control=5 control paradigms ###### \n")
cat("################################################################################ \n")

# Select subset of the data for analysis
data_all = data_control
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

boxplot(Percent_regret ~ Control, data = data_all)

lmer_model = lmer(Regret ~  Complexity * Control * Max_regret_group2 + (1|Subject),
                  data = data_all)

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

# Calculate anova
anov = Anova(lmer_model,type="III")
print(anov)

# Post-hoc tukey
dof = nrow(data_all)-5
print("degrees of freedom:")
print(dof)
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              covariate_average = TRUE),df=dof)))

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
cat("########################################################################### \n")
cat("####### THIS MODEL HAS A SINGULAR FIT ######### \n")
cat("########################################################################### \n")


# Select subset of the data for analysis
data_high = subset(data_control,Complexity=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

lmer_model = lmer(Regret ~ Control * Max_regret_group + (1|Subject),
                  data = data_high)
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
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              covariate_average = TRUE))))

cat("\n")
cat("########################################################################### \n")
cat("########################## LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
cat("########################################################################### \n")
cat("####### THIS MODEL HAS A SINGULAR FIT ######### \n")
cat("########################################################################### \n")


# Select subset of the data for analysis
data_low = subset(data_control,Complexity=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

lmer_model = lmer(Regret ~ Control * Max_regret_group + (1|Subject),
                  data = data_low)

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
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              covariate_average = TRUE))))

################################################################################
################################################################################
#               AUTONOMY
################################################################################
################################################################################

# define file to save data to
sink(paste(DIR,"Stats","Regret",paste("autonomy","Regret",skill,"lmer.txt",sep="-"),sep="/"))

cat("\n")
cat("########################################################################### \n")
cat("################################### ALL ################################### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_all = data_autonomy
data_all[] <- lapply(data_all, function(x) if(is.factor(x)) factor(x) else x)

lmer_model = lmer(Regret ~  Complexity * Control * Max_regret_group + (1|Subject),
                  data = data_all)

# plotting to evaluate assumptions
plot(lmer_model)

# Calculate anova
anov = Anova(lmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))

cat("\n")
cat("########################################################################### \n")
cat("######################### HIGH BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")
cat("########################################################################### \n")
cat("####### THIS MODEL HAS A SINGULAR FIT ######### \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_high = subset(data_autonomy,Complexity=='high')
data_high[] <- lapply(data_high, function(x) if(is.factor(x)) factor(x) else x)

lmer_model = lmer(Regret ~ Control * Max_regret_group + (1|Subject),
                  data = data_high)

# print("Sum of squares")
# sos = sum(resid(lmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos

# plotting to evaluate assumptions
plot(lmer_model)

# Calculate anova
anov = Anova(lmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))

cat("\n")
cat("########################################################################### \n")
cat("######################### LOW BUILDING DENSITY  ########################## \n")
cat("########################################################################### \n")

# Select subset of the data for analysis
data_low = subset(data_autonomy,Complexity=='low')
data_low[] <- lapply(data_low, function(x) if(is.factor(x)) factor(x) else x)

lmer_model = lmer(Regret ~  Control * Max_regret_group + (1|Subject),
                  data = data_low)

# print("Sum of squares")
# sos = sum(resid(lmer_model)^2)
# print(sos)
# modelerrorsum = modelerrorsum + sos
# print("Sum of all model error")
# print(modelerrorsum)

# plotting to evaluate assumptions
plot(lmer_model)

# Calculate anova
anov = Anova(lmer_model,type="III")
print(anov)

# Post-hoc tukey tests
print(summary(glht(lmer_model,
                   linfct=mcp(Control = "Tukey",
                              interaction_average = TRUE))))
