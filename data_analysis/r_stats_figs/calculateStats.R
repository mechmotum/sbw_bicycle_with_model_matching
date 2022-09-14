###
# This script runs all the statistical analysis included in the thesis.
# The data files "trials.csv" are "no_hands.csv" are required.
# They should be placed in the "data\" folder.
###
# The script saves the analysis data into the environment, with the naming
# convention of "analysis_type.area_of_interest", for example "welch.skill_dep"
# means Welch's test on skill dependency.
# All the data read from the data files and manipulated is stored in the 
# environment with names in CamelCase.
###
# Written by S. Drauksas, 2022
###
library(tidyverse)
library(janitor)
library(lmerTest)

# Welch, Baseline ----
Trials <- read_csv("data/trials.csv")
Trials$ID <- as.factor(Trials$ID)
Trials$Group <- as.factor(Trials$Group)
Trials$Trial <- as.factor(Trials$Trial)
# Normality tests (p = 0.7245, p = 0.9603)
shapiro.baseline_score <- shapiro.test(x = Trials[Trials$Group == 1 & Trials$Trial == "BL",]$Score - 
                                           Trials[Trials$Group == 2 & Trials$Trial == "BL",]$Score)
shapiro.baseline_var <- shapiro.test(x = Trials[Trials$Group == 1 & Trials$Trial == "BL",]$Variance - 
                                         Trials[Trials$Group == 2 & Trials$Trial == "BL",]$Variance)
# Welch tests (p = 0.6545, p = 0.958)
welch.baseline_score <- t.test(Score ~ Group,
                               data = Trials[Trials$Trial == "BL",],
                               paired = FALSE)
welch.baseline_var <- t.test(Variance ~ Group,
                             data = Trials[Trials$Trial == "BL",],
                             paired = FALSE)

# Welch, Performance Improvement ----
# Calculate improvement
PerfImp <- data.frame(matrix(ncol = 5, nrow = 10))
colnames(PerfImp) <- c("Group", "T1_Score", "T1_Var", "T2_Score", "T2_Var")
PerfImp$Group <- Trials[Trials$Trial == "BL",]$Group
PerfImp$T1_Score <- Trials[Trials$Trial == "T1",]$Score - 
                    Trials[Trials$Trial == "BL",]$Score
PerfImp$T2_Score <- Trials[Trials$Trial == "T2",]$Score - 
                    Trials[Trials$Trial == "MTR",]$Score
PerfImp$T1_Var <- Trials[Trials$Trial == "T1",]$Variance - 
                  Trials[Trials$Trial == "BL",]$Variance
PerfImp$T2_Var <- Trials[Trials$Trial == "T2",]$Variance - 
                  Trials[Trials$Trial == "MTR",]$Variance

# Welch tests (p = 0.9679, p = 0.08605, p = 0.8539, p = 0.07238)
welch.t1_score <- t.test(T1_Score ~ Group,
                         data = PerfImp,
                         paired = FALSE)
welch.t2_score <- t.test(T2_Score ~ Group,
                         data = PerfImp,
                         paired = FALSE)
welch.t1_var <- t.test(T1_Var ~ Group,
                       data = PerfImp,
                       paired = FALSE)
welch.t2_var <- t.test(T2_Var ~ Group,
                       data = PerfImp,
                       paired = FALSE)

# Wilcox, No-Hands ----
NH <- read_csv("data/no_hands.csv")
NH_NC <- NH[NH$scenario == "NH_NC",]
NH_C <- NH[NH$scenario == "NH_C",]
NH_C1 <- NH_C[1:20,]
NH_C1$scenario <- "NH_C1"
NH_C2 <- NH_C[21:40,]
NH_C2$scenario <- "NH_C2"
NH_C3 <- NH_C[41:60,]
NH_C3$scenario <- "NH_C3"

shapiro.NH_C1 <- shapiro.test(x = NH_C1$score - NH_NC$score)
shapiro.NH_C2 <- shapiro.test(x = NH_C2$score - NH_NC$score)
shapiro.NH_C3 <- shapiro.test(x = NH_C3$score - NH_NC$score)

NH <- rbind(NH_NC, NH_C1, NH_C2, NH_C3)

wilcox.no_hands <- pairwise.wilcox.test(x = NH$score,
                                        g = NH$scenario,
                                        paired = TRUE, 
                                        p.adjust.method = "bonferroni")

# Welch, Skill Dependency ----
SkillDep <- data.frame(matrix(ncol = 0, nrow = 20))
SkillDep$ID <- c(Trials[Trials$Group == 1 & Trials$Trial == "BL",]$ID, 
                 Trials[Trials$Group == 2 & Trials$Trial == "MTR",]$ID,
                 Trials[Trials$Group == 1 & Trials$Trial == "MTR",]$ID, 
                 Trials[Trials$Group == 2 & Trials$Trial == "BL",]$ID)
SkillDep$Skill <- c(Trials[Trials$Group == 1 & Trials$Trial == "BL",]$Score, 
                    Trials[Trials$Group == 2 & Trials$Trial == "MTR",]$Score,
                    Trials[Trials$Group == 1 & Trials$Trial == "MTR",]$Score, 
                    Trials[Trials$Group == 2 & Trials$Trial == "BL",]$Score)
SkillDep$Imp <- c(Trials[Trials$Group == 1 & Trials$Trial == "MTR",]$Score - 
  Trials[Trials$Group == 1 & Trials$Trial == "BL",]$Score, 
                  Trials[Trials$Group == 2 & Trials$Trial == "ETR",]$Score - 
  Trials[Trials$Group == 2 & Trials$Trial == "MTR",]$Score, 
                  Trials[Trials$Group == 1 & Trials$Trial == "ETR",]$Score - 
    Trials[Trials$Group == 1 & Trials$Trial == "MTR",]$Score, 
                  Trials[Trials$Group == 2 & Trials$Trial == "MTR",]$Score - 
    Trials[Trials$Group == 2 & Trials$Trial == "BL",]$Score)
SkillDep$Controller <- c(rep("On", 10), rep("Off", 10))

kmeans.skill_dep <- kmeans(x = cbind(SkillDep$Skill, SkillDep$Impr), 
                           centers = 2)

SkillDep$Cluster <- kmeans.skill_dep$cluster

welch.skill_dep <- t.test(Imp ~ Cluster,
                          data = SkillDep,
                          paired = FALSE)

# LME, Skill Dependency ----
lme.model <- lmer(Imp ~ Skill * Controller + (1 | ID),
                  data = SkillDep,
                  REML = FALSE)

anova.skill_dep <- anova(lme.model)