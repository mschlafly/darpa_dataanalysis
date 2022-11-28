# Imports
import csv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from utils.plot_utils import *

# def use_sub(required_trials,trial_happened_list, expertise, only_experts, only_novices):
def use_sub(required_trials,trial_happened_list):
    include_sub = False
    if np.sum(trial_happened_list)>=required_trials:
        # if only_experts:
        #     if expertise == "expert":
        #         include_sub = True
        # elif only_novices:
        #     if expertise == "novice":
        #         include_sub = True
        # else:
        include_sub = True
    return include_sub

def parse_file(file, Metrics, sub, environments, control, plot_each):

    data_mat = np.zeros((10,len(Metrics)+1))

    df = pd.read_csv(file)
    df_sub = df[df['Subject'] == sub]
    # print(df_sub)
    trial_num = 0
    for env in environments:
        for con in control:
            trial_i = df_sub[(df_sub['Control'] == con) & (df_sub['Complexity'] == env)].index.tolist()
            # print(trial_i)
            if len(trial_i)>0:
                flag_problem = False
                for met_i in range(len(Metrics)):
                    # print(Metrics[met_i],met_i,trial_i[0])
                    # print(df_sub[Metrics[met_i]].loc[trial_i[0]])
                    val = df_sub[Metrics[met_i]].loc[trial_i[0]]
                    if val=='na' or val=='nan':
                        flag_problem = True
                    if np.isnan(float(val)):
                        flag_problem = True
                    if Metrics[met_i]=='RR_Mean':
                        if abs(val)<1:
                            flag_problem = True

                if flag_problem==False:
                    for met_i in range(len(Metrics)):
                        data_mat[trial_num,met_i] = df_sub[Metrics[met_i]].loc[trial_i[0]]
                    data_mat[trial_num,-1] = 1
            # print(data_mat[trial_num,:])
            trial_num += 1

    if plot_each:
        width = 0.5
        ind = np.arange(10)
        for met_i in range(len(Metrics)):
            plt.figure(met_i)
            p1 = plt.bar(ind, data_mat[:,met_i], width)
            plt.ylabel(Metrics[met_i])
            plt.title(Metrics[met_i]+' for Subject ' + str(sub))
            labels = ('LN', 'LW', 'LD', 'LS', 'LA',
                      'HN', 'HW', 'HD', 'HS', 'HA')
            plt.xticks(ind, labels)
            plt.savefig('Plots/Indiv Plots/' + str(sub) + '_' + Metrics[met_i]+'.png')
        plt.close('all')

    return data_mat

def add_sub_to_lists(data_list, data_mat):
    num_metrics = data_mat.shape[1]-1
    # print('num_metrics',num_metrics)
    for i in range(10):
        if data_mat[i,-1] == 1:
            for met in range(num_metrics):
                # print(data_list[met])
                # print(data_list[met][i])
                if isinstance(data_list[met][i], int):
                    data_list[met][i] = np.array(data_mat[i,met])
                else:
                    # data_list[met][i].append(data_mat[i,met])
                    data_list[met][i] = np.append(data_list[met][i], data_mat[i,met])
    # print(data_list)
    return data_list

def return_metrics_for_trial(data_list, data_mat, i):
    metric_output = []
    num_metrics = data_mat.shape[1]-1
    for met in range(num_metrics):
        if data_mat[i,-1] == 1:
            metric_output.append(data_mat[i,met])
        else:
            metric_output.append(np.nan)
    return metric_output

def fnct_plot_score_paper(metrics_perf,data_list_perf,only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size):
    for met in range(len(metrics_perf)):
        if 'Lives' == metrics_perf[met]:
            lives_list = data_list_perf[met]
        if 'Treasure' == metrics_perf[met]:
            treasure_list = data_list_perf[met]

    combine_complexity = True
    if combine_complexity:
        data_low = lives_list[:5]
        data_high = lives_list[5:]
        data_lives = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data_lives.append(data_combined)
        data_low = treasure_list[:5]
        data_high = treasure_list[5:]
        data_treas = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data_treas.append(data_combined)
    else:
        data_lives = lives_list
        data_treas = treasure_list

    if only_experts:
        title = 'Game Performance'
        sig_matrix = np.array([
                    [1, 3, 0.0189], # waypoint - sharedergodic
                    [0,0,1]])
        spread_factor = 50

    elif only_novices:
        title = 'Game Performance'
        sig_matrix = np.array([
                    [2, 4, 0.0194],  # directergodic - autoergodic
                    [1, 4, 0.0438]])  # waypoint - autoergodic
        spread_factor = 20
    else:
        title = 'Game Performance'
        sig_matrix = np.array([])
        spread_factor = 30

    xlabel = ''
    ylabel = 'Final Game Score'
    data_lives2 = [i * 3 for i in data_lives]
    [fig, ax, upper_data_bound] = make_stackedbar(data_lives2,
                                                  data_treas,
                                                  title, xlabel, ylabel,
                                                  labels5, colors5, alphas5,
                                                  figure_size)

    if only_novices:
        ax.set_ylim(bottom=17, top=28.5)
        y = 15
    else:
        ax.set_ylim(bottom=18, top=28.5)
        y = 17
    add_stats(upper_data_bound, sig_matrix, ax, spread_factor=spread_factor, type='bar')

    # Add arrows on the bottom of the plot
    fig.subplots_adjust(bottom=0.18)
    text_buffer = .65

    x1 = [1.75, 0]  # start of the arrow
    x2 = [4.25, 0]  # end of the arrow
    name = ['Coverage Control', '']  # single label due to combining env. complexity
    add_labels(ax, x1, x2, y, name, text_buffer)

    if only_experts:
        plt.savefig(file_plot + 'Score/score_experts.pdf')
    elif only_novices:
        plt.savefig(file_plot + 'Score/score_novices.pdf')
    else:
        plt.savefig(file_plot + 'Score/score.pdf')

def fnct_plot_input(metrics_perf,data_list_perf,only_experts,only_novices,file_plot,figure_size):
    alphas_input = [None, None, None]
    colors_input = ['#BA0071','#0071BA','#00BA49']
    labels_input = ['1-Robot\nCommands','3-Robot\nCommands','Shared\nControl']

    for met in range(len(metrics_perf)):
        if 'Input' == metrics_perf[met]:
            input_list = data_list_perf[met]

    combine_complexity = True
    if combine_complexity:
        data_low = input_list[:5]
        data_high = input_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
        data = data[1:4]
    else:
        data = input_list

    # Add stastical significant basedon ANOVA #################################
    if only_experts:
        title = 'Instructions Required to Operate Swarm'
        sig_matrix = np.array([
                    [1, 2, 0.002745709],  # direct ergodic - shared ergodic
                    [0, 2, 0.001252248]])  # waypoint - shared ergodic
    elif only_novices:
        title = 'Instructions Required to Operate Swarm'
        sig_matrix = np.array([
                    [0, 2, 0.03525669]])  # waypoint - shared ergodic
    else:
        title = 'Instructions Required to Operate Swarm'
        sig_matrix = np.array([
                    [1, 2, 0.0005034832],  # direct ergodic - shared ergodic
                    [0, 2, 4.593629e-05]])  # waypoint - shared ergodic

    # Create a plot ###########################################################
    xlabel = ''
    ylabel = 'Number of Commands During Trial'
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels_input, colors_input)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=18,type='bar')

    fig.subplots_adjust(bottom=0.23,left=.18)  # asjust white spacing on the bottom
    # Add arrows on the bottom of the plot

    # fig.subplots_adjust(bottom=0.18)
    # text_buffer = .3

    # x1 = [1.75, 0]  # start of the arrow
    # # x1 = [.75, 0]  # start of the arrow
    # x2 = [4.25, 0]  # end of the arrow
    # # x2 = [2.25, 0]  # end of the arrow
    # y = 4
    # name = ['Coverage Control', '']  # single label due to combining env. complexity
    # add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################
    if only_experts:
        fig.savefig(file_plot + 'Inputs/inputs_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'Inputs/inputs_novices.pdf')
    else:
        fig.savefig(file_plot + 'Inputs/inputs.pdf')

def fnct_plot_difficulty(metrics,data_list,only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size):
    for met in range(len(metrics)):
        if 'Difficulty' == metrics[met]:
            difficulty_list = data_list[met]

    combine_complexity = True
    if combine_complexity:
        data_low = difficulty_list[:5]
        data_high = difficulty_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = difficulty_list

    if only_experts:
        title = 'Experienced Participants\' Difficulty Rating'
        sig_matrix = np.array([
                        [1, 4, 0.0278],  # waypoint - auto ergodic
                        [0, 4, 0.0113]])  # none - auto ergodic
        y = 6.25
        text_buffer = .15
    elif only_novices:
        title = 'Novice Participants\' Difficulty Rating'
        sig_matrix = np.array([
                        [1, 2, 0.0223]])  # waypoint - direct ergodic
        y = 4.9
        text_buffer = .2
    else:
        title = 'Perceived Difficulty Rating'
        sig_matrix = np.array([
                        [1, 4, 0.0166],  # waypoint - autoergodic
                        [0, 2, 0.0298],  # none - directergodic
                        [0, 3, 0.0133],  # none - sharedergodic
                        [0, 3, 0.0014]])  # none - autoergodic
        y = 6.35
        text_buffer = .15

    # Create a plot ###########################################################
    xlabel = ''
    ylabel = 'Ease of Usage'
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels5, colors5)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')

    # Add *Ergodic* label and arrow on the bottom of the plot #################
    fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom

    x1 = [1.75, 0]  # start of the arrow
    x2 = [4.25, 0]  # end of the arrow
    name = ['Coverage Control', '']  # single label due to combining env. complexity

    add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################

    if only_experts:
        fig.savefig(file_plot + 'Difficulty/difficulty_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'Difficulty/difficulty_novices.pdf')
    else:
        fig.savefig(file_plot + 'Difficulty/difficulty.pdf')

def fnct_plot_RR(metrics,data_list,only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size):

    # figure_size = (4.5, 3.25)
    # colors_RR = ['#BA4900','#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
    # colors_RR = ['#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
    # # labels_RR = ['','No Swarm','Waypoint\nControl','User','Shared','Autonomous']
    # labels_RR = ['','Baseline','1-Robot\nCommands','3-Robot\nCommands','Shared\nControl','Fully\nAutonomous']


    for met in range(len(metrics)):
        if 'RR_Zscore' == metrics[met]:
            RR_list = data_list[met]

    combine_complexity = True
    if combine_complexity:
        data_low = RR_list[:5]
        data_high = RR_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = -np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = RR_list

    if only_experts:
        title = '\'RR\' Interval'
        sig_matrix = np.array([
                                [0,1,0.02533904], # none - wp
                                [1,3,0.03165469],  # wp - shared
                                [1,4,0.008010422]  # wp - auto
                                ])
        y = -1
    elif only_novices:
        title = '\'RR\' Interval'
        sig_matrix = np.array([])
        y = -1.1
    else:
        title = '\'RR\' Interval'
        sig_matrix = np.array([])
        y = -.7

    # Create a plot ##########################################################
    ylabel = 'Within-Subject Z-score'
    xlabel = ''
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels5, colors5)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=20,type='bar')

    # Add *Ergodic* label and arrow on the bottom of the plot #################
    # fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom
    fig.subplots_adjust(bottom=0.23,left=.18)  # asjust white spacing on the bottom

    # x1 = [2.75, 0]  # start of the arrow
    # x2 = [5.25, 0]  # end of the arrow
    # text_buffer = .1
    # name = ['Coverage Control', '']  # single label due to combining env. complexity
    #
    # add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################

    if only_experts:
        fig.savefig(file_plot + 'RR/RR_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'RR/RR_novices.pdf')
    else:
        fig.savefig(file_plot + 'RR/RR.pdf')

def fnct_plot_POMDP_d(metrics,data_list,only_experts,only_novices,file_plot):

    figure_size = (4.5, 3.25)
    colors_RR = ['#BA4900','#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
    labels_RR = ['','No Swarm','Waypoint\nControl','User','Shared','Autonomous']

    for met in range(len(metrics)):
        if 'Perc_regret_.1' == metrics[met]:
            RR_list = data_list[met]

    combine_complexity = True
    if combine_complexity:
        data_low = RR_list[:5]
        data_high = RR_list[5:]
        data = [np.array([0.3,0.3,0.3])]

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = RR_list

    if only_experts:
        title = 'Decision Regret'
        sig_matrix = np.array([
                                [1,2,0.0243], # none - wp
                                [3,4,0.0009],  # direct - shared
                                [2,4,0.0009],  # wp - shared
                                [3,5,0.0009],  # direct - auto
                                [2,5,0.0009],  # wp - auto
                                ])
        y = -1
    elif only_novices:
        print('Error')
    else:
        print('Error')

    # Create a plot ##########################################################
    ylabel = 'Percent Regret Compared to Optimal Agent'
    xlabel = ''
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels_RR, colors_RR)

    # Add stastical signicant marking #########################################
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')

    # # Add *Ergodic* label and arrow on the bottom of the plot #################
    # fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom
    #
    # x1 = [2.75, 0]  # start of the arrow
    # x2 = [5.25, 0]  # end of the arrow
    # text_buffer = .1
    # name = ['Coverage Control', '']  # single label due to combining env. complexity

    # add_labels(ax, x1, x2, y, name, text_buffer)

    # Saving the plots ########################################################

    if only_experts:
        fig.savefig(file_plot + 'POMDP/POMDP_d_experts.pdf')

def plot_metric(metric_name,data_list,only_experts,only_novices,file_plot,labels5, colors5, alphas5,figure_size):


    # for plotting with partial data
    for i in range(10):
        if isinstance(data_list[i], int):
            data_list[i] = np.array([data_list[i],data_list[i]])
        elif isinstance(data_list[i], float):
            data_list[i] = np.array([0,0])

    combine_complexity = True
    if combine_complexity:
        data_low = data_list[:5]
        data_high = data_list[5:]
        data = []

        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
    else:
        data = data_list


    # Create a plot ###########################################################
    title = metric_name
    xlabel = ''
    ylabel = metric_name
    fig, ax = make_boxplot(data, title, xlabel, ylabel, labels5, colors5, alphas5, figure_size)

    # Saving the plots ########################################################
    fig.savefig(file_plot + metric_name+'.png')

def fnct_plot_N_obs(only_experts,only_novices,file_plot,labels4,colors4,alphas4,figure_size):

    df_raw_formatted = pd.read_csv('raw_data_formatted/raw_data_formatted.csv')
    df_N_obs = df_raw_formatted[(df_raw_formatted['Include_N_obs']==True) &
                                (df_raw_formatted['Control']!='none')]

    # Set up figure parameters
    title = 'Robotic Performance\nat Detecting People'
    xlabel = ''
    ylabel = 'Number of People Found\nDuring 5min Trial'

    if only_experts:
        df_N_obs = df_N_obs[df_N_obs['Expertise']=='expert']
        sig_matrix = np.array([
                                [0,1,0.0009],  # wp - direct
                                [1,2,0.0009],  # direct - shared
                                [2,3,0.017],  # shared - auto
                                [0,2,0.0009],  # wp - shared
                                [1,3,0.0009],  # direct - auto
                                [0,3,0.0009]  # wp - auto
                                ])
    elif only_novices:
        df_N_obs = df_N_obs[df_N_obs['Expertise']=='novice']
        sig_matrix = np.array([
                                [0,1,0.003],  # wp - direct
                                [1,2,0.0078],  # direct - shared
                                [2,3,1],  # shared - auto
                                [0,2,0.0009],  # wp - shared
                                [1,3,0.0017],  # direct - auto
                                [0,3,0.0009]  # wp - auto
                                ])
    else:
        sig_matrix = np.array([
                                [0,1,0.0009],  # wp - direct
                                [1,2,0.0009],  # direct - shared
                                [2,3,0.0413],  # shared - auto
                                [0,2,0.0009],  # wp - shared
                                [1,3,0.0009],  # direct - auto
                                [0,3,0.0009]  # wp - auto
                                ])

    # Plot data
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    sns.barplot(data=df_N_obs, x="Control", y="N_observations", palette = colors4, saturation = alphas4,
                ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.1)
    # sns.stripplot(data=df_N_obs, x="Control", y="N_observations", ax=ax, size=2, color='black') #jitter=.2,

    # Add titles and labels
    plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
    plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
    plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    ax.set_xticklabels(labels4, fontname="sans-serif", fontsize=9)
    for label in (ax.get_yticklabels()):
        label.set_fontsize(8)
    for tick in ax.get_xticklabels():
        tick.set_rotation(30)

    # Add stastical signicant marking #########################################

    upper_data_bound = []
    control_options = ['waypoint','directergodic','sharedergodic','autoergodic']
    for i in range(len(control_options)):
        df_temp = df_N_obs[df_N_obs['Control']==control_options[i]]
        upper_data_bound.append(df_temp['N_observations'].mean())
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
    # ax.set_ylim([.25,.64])

    fig.subplots_adjust(bottom=0.15)  # asjust white spacing

    if only_experts:
        fig.savefig(file_plot + 'N_found/n_observations_experts.pdf')
    elif only_novices:
        fig.savefig(file_plot + 'N_found/n_observations_novices.pdf')
    else:
        fig.savefig(file_plot + 'N_found/n_observations.pdf')

def fnct_plot_POMDP_obs(only_experts,only_novices,file_plot,labels4,colors4,alphas4,figure_size):

    df = pd.read_csv('raw_data/raw_data_POMDP_obs_i_6a_10000s.csv')
    # df = pd.read_csv('raw_data/raw_data_POMDP_obs_cum_6a_10000s.csv')
    metric = "P_switch"
    # metric = "P_switch_cum"

    # Set up figure parameters
    title = 'Usefulness of Robotic\nBehavior to Human'
    xlabel = ''
    ylabel = 'Probability Optimal Path Changes\nDue to New Information'
    sig_matrix = np.array([])


    # Plot data
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    sns.barplot(data=df, x="Control", y=metric, palette = colors4, saturation = alphas4,
                ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.1)
    # sns.stripplot(data=df, x="Control", y="P_switch", ax=ax, size=2, color='black') #jitter=.2,

    # Add titles and labels
    plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
    plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
    plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    ax.set_xticklabels(labels4, fontname="sans-serif", fontsize=9)
    for label in (ax.get_yticklabels()):
        label.set_fontsize(8)
    for tick in ax.get_xticklabels():
        tick.set_rotation(30)

    # Add stastical signicant marking #########################################

    upper_data_bound = []
    control_options = ['waypoint','directergodic','sharedergodic','autoergodic']
    for i in range(len(control_options)):
        df_temp = df[df['Control']==control_options[i]]
        upper_data_bound.append(df_temp[metric].mean())
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
    ax.set_ylim([.25,.4])

    fig.subplots_adjust(bottom=0.15,left=.2)  # asjust white spacing

    fig.savefig(file_plot + 'Observations/p_switch_experts.pdf')
    # if only_experts:
    #     fig.savefig(file_plot + 'N_found/n_observations_experts.pdf')
    # elif only_novices:
    #     fig.savefig(file_plot + 'N_found/n_observations_novices.pdf')
    # else:
    #     fig.savefig(file_plot + 'N_found/n_observations.pdf')

def fnct_plot_performance(only_experts,only_novices,file_plot,labels5,colors5,alphas5,figure_size):

    df_raw_formatted = pd.read_csv('raw_data_formatted/raw_data_formatted.csv')
    df = df_raw_formatted[(df_raw_formatted['Include_Score']==True)]
    df['Lives*3'] = df['Lives'].values * 3

    # Set up figure parameters
    title = 'Game Performance'
    xlabel = ''
    ylabel = 'Final Game Score'

    if only_experts:
        df = df[df['Expertise']=='expert']
        sig_matrix = np.array([])
    elif only_novices:
        df = df[df['Expertise']=='novice']
        sig_matrix = np.array([
                    [2, 4, 0.0194],  # directergodic - autoergodic
                    [1, 4, 0.0438]])  # waypoint - autoergodic
    else:
        sig_matrix = np.array([])

    # Plot data
    fig, ax = plt.subplots(figsize=figure_size,dpi=300)
    sns.barplot(data=df, x="Control", y="Score", palette = colors5,
                ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.1, hatch='xx')
    sns.barplot(data=df, x="Control", y="Lives*3", palette = colors5,
                ax=ax, ci=None)
    # sns.stripplot(data=df_N_obs, x="Control", y="N_observations", ax=ax, size=2, color='black') #jitter=.2,

    # Add titles and labels
    plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
    plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
    plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    ax.set_xticklabels(labels5, fontname="sans-serif", fontsize=9)
    for label in (ax.get_yticklabels()):
        label.set_fontsize(8)
    for tick in ax.get_xticklabels():
        tick.set_rotation(30)

    # Add stastical signicant marking #########################################
    upper_data_bound = []
    control_options = ['none','waypoint','directergodic','sharedergodic','autoergodic']
    for i in range(len(control_options)):
        df_temp = df[df['Control']==control_options[i]]
        val = df_temp['N_observations'].mean() + df_temp['N_observations'].sem()
        upper_data_bound.append(val)
    add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
    # ax.set_ylim([.25,.64])

    fig.subplots_adjust(bottom=0.15)  # asjust white spacing

    if only_experts:
        ax.set_ylim(bottom=18, top=27)
        plt.savefig(file_plot + 'Score/score_bar_experts.pdf')
    elif only_novices:
        ax.set_ylim(bottom=17, top=27)
        plt.savefig(file_plot + 'Score/score_bar_novices.pdf')
    else:
        plt.savefig(file_plot + 'Score/score_bar.pdf')




# def parse_performance_data(file,subID,environments,control,plot_each):
#
#     lives = np.zeros(10)
#     treasure = np.zeros(10)
#     input = np.zeros(10)
#     difficulty = np.zeros(10)
#     score = np.zeros(10)
#     trial_happened = np.zeros(10)
#
#     with open(file, 'r') as csvfile:
#         data = csv.reader(csvfile, delimiter=',')
#
#         # Loop through rows in csv with each row representing a trial.
#         # If the row is for the subject of interest, act
#         for row in data:
#             if row[0] == subID:
#                 # If the row is for a low complexity trial,
#                 # store data in the first 5 columns
#                 if row[2] == environments[0]:
#                     for i in range(len(control)):
#                         if row[1] == control[i]:
#                             lives[i] = row[3]
#                             treasure[i] = row[4]
#                             input[i] = row[5]
#                             difficulty[i] = row[6]
#                             score[i] = lives[i]*3. + treasure[i]
#                             trial_happened[i] = 1
#                 # If the row is for a high complexity trial,
#                 # store data in the last 5 columns
#                 else:
#                     for i in range(len(control)):
#                         if row[1] == control[i]:
#                             lives[i+5] = row[3]
#                             treasure[i+5] = row[4]
#                             input[i+5] = row[5]
#                             difficulty[i+5] = row[6]
#                             score[i+5] = lives[i+5]*3. + treasure[i+5]
#                             trial_happened[i+5] = 1
#
#     # Plot bar graph for individual subject
#     if plot_each:
#         plt.figure(0)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, lives, width)
#         p2 = plt.bar(ind, treasure, width, bottom=lives)
#         plt.ylabel('Score')
#         plt.title('Game Performance for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.legend((p1[0], p2[0]), ('Lives', 'Targets'))
#         plt.savefig('Plots/Indiv Plots/' + subID + '_performance.png')
#
#         plt.figure(0+1)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, input, width)
#         plt.ylabel('Score')
#         plt.title('Inputs for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_input.png')
#
#         plt.figure(0+2)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, difficulty, width)
#         plt.ylabel('Score')
#         plt.title('Difficulty for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_difficulty.png')
#         plt.close('all')
#
#     return [lives, treasure, input, difficulty, score, trial_happened]
#
# def parse_cogload_data(file,subID,environments,control,plot_each):
#
#     subID_RR = 'Sub'+subID
#     RR_mean = np.zeros(10)
#     RR_zscore = np.zeros(10)
#     trial_happened = np.zeros(10)
#
#     with open(file, 'r') as csvfile:
#         data = csv.reader(csvfile, delimiter=',')
#
#         # Loop through rows in csv with each row representing a trial.
#         # If the row is for the subject of interest, act
#         for row in data:
#             # print(row)
#             if row[0] == subID_RR:
#                 # If the row is for a low complexity trial,
#                 # store data in the first 5 columns
#                 if row[2] == environments[0]:
#                     for i in range(len(control)):
#                         if row[1] == control[i]:
#                             if float(row[8]) > 0:
#                                 RR_zscore[i] = row[9]#(1/float(row[8]))*1000*60
#                                 RR_mean[i] = row[8]
#                                 trial_happened[i] = 1
#                 # If the row is for a high complexity trial,
#                 # store data in the last 5 columns
#                 else:
#                     for i in range(len(control)):
#                         if row[1] == control[i]:
#                             if float(row[8]) > 0:
#                                 RR_zscore[i+5] = row[9]#(1/float(row[8]))*1000*60
#                                 RR_mean[i+5] = row[8]
#                                 trial_happened[i+5] = 1
#
#     # Plot bar graph for individual subject
#     if plot_each:
#         plt.figure(0)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, RR_mean, width)
#         plt.ylabel('Mean RR Interval')
#         plt.title('RR Interval for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_RR.png')
#         plt.close('all')
#
#     return [RR_mean, RR_zscore, trial_happened]
#
# def parse_MDP_data(file,sub,environments,control,plot_each):
#
#     regret_cum = np.zeros(10)
#     trial_happened = np.zeros(10)
#
#     with open(file, 'r') as csvfile:
#         data = csv.reader(csvfile, delimiter=',')
#
#         # Loop through rows in csv with each row representing a trial.
#         # If the row is for the subject of interest, act
#         for row in data:
#             if row[0] == str(sub):
#                 # If the row is for a low complexity trial,
#                 # store data in the first 5 columns
#                 if row[2] == environments[0]:
#                     for i in range(len(control)):
#                         if row[1] == control[i]:
#                             if not np.isnan(float(row[3])):
#                                 trial_happened[i] = 1
#                                 regret_cum[i] = row[3]
#                 # If the row is for a high complexity trial,
#                 # store data in the last 5 columns
#                 else:
#                     for i in range(len(control)):
#                         if row[1] == control[i]:
#                             if not np.isnan(float(row[3])):
#                                 regret_cum[i+5] = row[3]
#                                 trial_happened[i+5] = 1
#
#     # Plot bar graph for individual subject
#     if plot_each:
#         plt.figure(0)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, regret_cum, width)
#         plt.ylabel('Cummulative Regret')
#         plt.title('Cummulative Regret for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_regret.png')
#         plt.close('all')
#
#     return [regret_cum, trial_happened]
#
# def parse_POMDP_data(file,sub,environments,control,plot_each):
#
#     regret_mean0 = np.zeros(10)
#     regret_mean1 = np.zeros(10)
#     regret_percent0 = np.zeros(10)
#     regret_percent1 = np.zeros(10)
#     n_observations = np.zeros(10)
#     norm_mean = np.zeros(10)
#     norm_cum = np.zeros(10)
#     trial_happened_n = np.zeros(10)
#     trial_happened_regret0 = np.zeros(10)
#     trial_happened_regret1 = np.zeros(10)
#     trial_happened_norm = np.zeros(10)
#
#     with open(file, 'r') as csvfile:
#         data = csv.reader(csvfile, delimiter=',')
#
#         # Loop through rows in csv with each row representing a trial.
#         # If the row is for the subject of interest, act
#         for row in data:
#             if row[0]!='Subject':
#                 if int(row[0]) == sub:
#                     # If the row is for a low complexity trial,
#                     # store data in the first 5 columns
#                     if row[2] == environments[0]:
#                         for i in range(len(control)):
#                             if row[1] == control[i]:
#                                 n_observations[i] = row[3]
#                                 trial_happened_n[i] = 1
#                                 if row[6]!='na':
#                                     if not np.isnan(float(row[6])):
#                                         regret_mean0[i] = row[6]
#                                         regret_percent0[i] = row[7]
#                                         trial_happened_regret0[i] = 1
#                                 if row[8]!='na':
#                                     if not np.isnan(float(row[8])):
#                                         regret_mean1[i] = row[8]
#                                         regret_percent1[i] = row[9]
#                                         trial_happened_regret1[i] = 1
#                                 if row[4]!='na':
#                                     if not np.isnan(float(row[4])):
#                                         norm_mean[i] = row[4]
#                                         norm_cum[i] = row[5]
#                                         trial_happened_norm[i] = 1
#                                     # if row[4]!='na':
#                                     #     if not np.isnan(float(row[4])):
#                                     #         regret_mean[i] = row[4]
#                                     #         regret_percent[i] = row[5]
#                                     #         trial_happened_regret[i] = 1
#                                     # if row[6]!='na':
#                                     #     if not np.isnan(float(row[6])):
#                                     #         norm_mean[i] = row[6]
#                                     #         norm_cum[i] = row[7]
#                                     #         trial_happened_norm[i] = 1
#                     # If the row is for a high complexity trial,
#                     # store data in the last 5 columns
#                     else:
#                         for i in range(len(control)):
#                             if row[1] == control[i]:
#                                 n_observations[i+5] = row[3]
#                                 trial_happened_n[i+5] = 1
#                                 if row[6]!='na':
#                                     if not np.isnan(float(row[6])):
#                                         regret_mean0[i+5] = row[6]
#                                         regret_percent0[i+5] = row[7]
#                                         trial_happened_regret0[i+5] = 1
#                                 if row[8]!='na':
#                                     if not np.isnan(float(row[8])):
#                                         regret_mean1[i+5] = row[8]
#                                         regret_percent1[i+5] = row[9]
#                                         trial_happened_regret1[i+5] = 1
#                                 if row[4]!='na':
#                                     if not np.isnan(float(row[4])):
#                                         norm_mean[i+5] = row[4]
#                                         norm_cum[i+5] = row[5]
#                                         trial_happened_norm[i+5] = 1
#
#                                 # if row[4]!='na':
#                                 #     if not np.isnan(float(row[4])):
#                                 #         regret_mean[i+5] = row[4]
#                                 #         regret_percent[i+5] = row[5]
#                                 #         trial_happened_regret[i+5] = 1
#                                 # if row[6]!='na':
#                                 #     if not np.isnan(float(row[6])):
#                                 #         norm_mean[i+5] = row[6]
#                                 #         norm_cum[i+5] = row[7]
#                                 #         trial_happened_norm[i+5] = 1
#     # Plot bar graph for individual subject
#     if plot_each:
#         plt.figure(0)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, regret_mean, width)
#         plt.ylabel('Mean Regret Based on Known Information')
#         plt.title('Regret for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_regret_knowninfo.png')
#
#
#         plt.figure(1)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, regret_percent, width)
#         plt.ylabel('Percent Regret Based on Known Information')
#         plt.title('Regret for Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_regret_knowninfo_perc.png')
#
#         plt.figure(2)
#         width = 0.5
#         ind = np.arange(10)
#         p1 = plt.bar(ind, n_observations, width)
#         plt.ylabel('Number of Drone Observations')
#         plt.title('Subject ' + subID)
#         labels = ('LN', 'LW', 'LD', 'LS', 'LA',
#                   'HN', 'HW', 'HD', 'HS', 'HA')
#         plt.xticks(ind, labels)
#         plt.savefig('Plots/Indiv Plots/' + subID + '_drone_observations.png')
#
#         plt.close('all')
#
#     # return [regret_mean, regret_percent, n_observations, norm_mean, norm_cum, trial_happened_n, trial_happened_regret, trial_happened_norm]
#     return [regret_mean0, regret_percent0, regret_mean1, regret_percent1, n_observations, norm_mean, norm_cum, trial_happened_n, trial_happened_regret0, trial_happened_regret1, trial_happened_norm]
