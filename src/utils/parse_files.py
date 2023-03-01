# Imports
import csv
import matplotlib.pyplot as plt
import matplotlib.colors
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
            trial_i = df_sub[(df_sub['Control'] == con) & (df_sub['Density'] == env)].index.tolist()
            flag_problem = False
            for met_i in range(len(Metrics)):
                if len(trial_i)>1:
                    # print(env,con)
                    # print(trial_i)
                    # print(df_sub.loc[trial_i])
                    vals_all = df_sub[Metrics[met_i]].loc[trial_i]
                    val = np.mean(vals_all.values)
                    # print(val)
                    # if Metrics[met_i]=='P_switch':
                    #     print(len(trial_i))
                    # oasihd
                elif len(trial_i)>0:
                    val = df_sub[Metrics[met_i]].loc[trial_i[0]]
                else:
                    val = 'nan'
                if val=='na' or val=='nan':
                    flag_problem = True
                elif np.isnan(float(val)):
                    flag_problem = True
                elif Metrics[met_i]=='RR_Mean':
                    if val<1:
                        flag_problem = True

                if flag_problem==False:
                    # for met_i in range(len(Metrics)):
                    data_mat[trial_num,met_i] = val
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
            labels = ('LB', 'LW', 'LU', 'LS', 'LA',
                      'HB', 'HW', 'HU', 'HS', 'HA')
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

def fnct_plot_score_paper(metrics_perf,data_list_perf,only_experts,only_novices,combine_environments,file_plot,labels5, colors5, alphas5,figure_size):
    for met in range(len(metrics_perf)):
        if 'Lives' == metrics_perf[met]:
            lives_list = data_list_perf[met]
        if 'Treasure' == metrics_perf[met]:
            treasure_list = data_list_perf[met]

    # data_low = RR_list[:5]
    # data_high = RR_list[5:]
    if combine_environments:
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
        num_environments = 1
    else:
        num_environments = 2
        # data_lives = lives_list
        # data_treas = treasure_list


    for plot_num in range(num_environments):
        if not combine_environments:
            if plot_num==0:
                data_lives = lives_list[:5]
                data_treas = treasure_list[:5]
            else:
                data_lives = lives_list[5:]
                data_treas = treasure_list[5:]

        title = 'Game Performance'
        if only_experts:
            if combine_environments:
                sig_matrix = np.array([])
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                [3, 4, 0.0275], # shared - auto
                                [0,0,1]])
                else:
                    sig_matrix = np.array([])
            spread_factor = 50
        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([
                            [1, 4, 0.0427], # waypoint - auto
                            [0,0,1]])
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([
                                [1, 4, 0.0186], # waypoint - auto
                                [0,0,1]])
            spread_factor = 20
        else:
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

        # Saving the plots ########################################################
        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'Score/score_experts.pdf')
                fig.savefig(file_plot + 'Score/score_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Score/score_experts_lowD.pdf')
                    fig.savefig(file_plot + 'Score/score_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'Score/score_experts_highD.pdf')
                    fig.savefig(file_plot + 'Score/score_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'Score/score_novices.pdf')
                fig.savefig(file_plot + 'Score/score_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Score/score_novices_lowD.pdf')
                    fig.savefig(file_plot + 'Score/score_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'Score/score_novices_highD.pdf')
                    fig.savefig(file_plot + 'Score/score_novices_highD.png')
        else:
            fig.savefig(file_plot + 'Score/score.pdf')
            fig.savefig(file_plot + 'Score/score.png')

def fnct_plot_input(metrics_perf,data_list_perf,only_experts,only_novices,combine_environments,file_plot,figure_size):
    alphas_input = [None, None, None]
    colors_input = ['#BA0071','#0071BA','#00BA49']
    # labels_input = ['1-Robot\nCommands','3-Robot\nCommands','Shared\nControl']
    labels_input = ['Waypoint','User','Shared\nControl']

    for met in range(len(metrics_perf)):
        if 'Input' == metrics_perf[met]:
            input_list = data_list_perf[met]

    data_low = input_list[:5]
    data_high = input_list[5:]
    if combine_environments:
        data = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):
        if not combine_environments:
            if plot_num==0:
                data = data_low
            else:
                data = data_high
        data = data[1:4]

        # Add stastical significant basedon ANOVA #################################
        title = 'Instructions Required to Operate Robots'
        if only_experts:
            if combine_environments:
                sig_matrix = np.array([
                            [1, 2, 0.002745709],  # direct ergodic - shared ergodic
                            [0, 2, 0.001252248]])  # waypoint - shared ergodic
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                [0, 2, 0.049999]])  # waypoint - shared ergodic
                else:
                    sig_matrix = np.array([
                                [1, 2, 0.038],  # direct ergodic - shared ergodic
                                [0, 2, 0.032]])  # waypoint - shared ergodic

        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([
                            [0, 2, 0.049]])  # waypoint - shared ergodic
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([])
        else:
            sig_matrix = np.array([
                        [1, 2, 0.00064],  # direct ergodic - shared ergodic
                        [0, 2, 6.028342e-05]])  # waypoint - shared ergodic

        # Create a plot ###########################################################
        xlabel = ''
        ylabel = 'Number of Commands During Trial'
        fig, ax = plt.subplots(figsize=figure_size,dpi=300)
        upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels_input, colors_input)

        # Add stastical signicant marking #########################################
        add_stats(upper_data_bound,sig_matrix,ax,spread_factor=18,type='bar')

        # fig.subplots_adjust(bottom=0.23,left=.2)  # asjust white spacing on the bottom
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
            if combine_environments:
                fig.savefig(file_plot + 'Inputs/inputs_experts.pdf')
                fig.savefig(file_plot + 'Inputs/inputs_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Inputs/inputs_experts_lowD.pdf')
                    fig.savefig(file_plot + 'Inputs/inputs_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'Inputs/inputs_experts_highD.pdf')
                    fig.savefig(file_plot + 'Inputs/inputs_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'Inputs/inputs_novices.pdf')
                fig.savefig(file_plot + 'Inputs/inputs_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Inputs/inputs_novices_lowD.pdf')
                    fig.savefig(file_plot + 'Inputs/inputs_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'Inputs/inputs_novices_highD.pdf')
                    fig.savefig(file_plot + 'Inputs/inputs_novices_highD.png')
        else:
            fig.savefig(file_plot + 'Inputs/inputs.pdf')
            fig.savefig(file_plot + 'Inputs/inputs.png')

def fnct_plot_difficulty(metrics,data_list,only_experts,only_novices,combine_environments,file_plot,labels5, colors5, alphas5, figure_size):
    for met in range(len(metrics)):
        if 'Difficulty' == metrics[met]:
            difficulty_list = data_list[met]

    data_low = difficulty_list[:5]
    data_high = difficulty_list[5:]
    if combine_environments:
        data = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):
        if not combine_environments:
            if plot_num==0:
                data = data_low
            else:
                data = data_high

        if only_experts:
            if combine_environments:
                sig_matrix = np.array([
                            [1, 2, 0.018],  # waypoint - direct
                            [0, 4, 0.044]  # none - auto
                            ])  # waypoint - shared ergodic
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([])

        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([])
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([])
        else:
            sig_matrix = np.array([
                        [0, 2, 0.029],  # none - direct
                        [0, 3, 0.005],  # none - shared
                        [1, 4, 0.02],  # waypoint - auto
                        [0, 4, 0.009]  # none - auto
                        ])  # waypoint - shared ergodic

        title = 'Perceived Difficulty Rating'

        # Create a plot ###########################################################
        xlabel = ''
        ylabel = 'Ease of Usage'
        fig, ax = plt.subplots(figsize=figure_size,dpi=300)
        upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels5, colors5)

        # Add stastical signicant marking #########################################
        add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
        fig.subplots_adjust(bottom=0.23,left=.18)  # asjust white spacing on the bottom

        # Saving the plots ########################################################
        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'Difficulty/difficulty_experts.pdf')
                fig.savefig(file_plot + 'Difficulty/difficulty_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Difficulty/difficulty_experts_lowD.pdf')
                    fig.savefig(file_plot + 'Difficulty/difficulty_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'Difficulty/difficulty_experts_highD.pdf')
                    fig.savefig(file_plot + 'Difficulty/difficulty_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'Difficulty/difficulty_novices.pdf')
                fig.savefig(file_plot + 'Difficulty/difficulty_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Difficulty/difficulty_novices_lowD.pdf')
                    fig.savefig(file_plot + 'Difficulty/difficulty_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'Difficulty/difficulty_novices_highD.pdf')
                    fig.savefig(file_plot + 'Difficulty/difficulty_novices_highD.png')
        else:
            fig.savefig(file_plot + 'Difficulty/difficulty.pdf')
            fig.savefig(file_plot + 'Difficulty/difficulty.png')

def fnct_plot_RR(metrics,data_list,only_experts,only_novices,combine_environments,file_plot,labels5, colors5, alphas5,figure_size):

    # figure_size = (4.5, 3.25)
    # colors_RR = ['#BA4900','#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
    # colors_RR = ['#BA4900','#BA0071','#0071BA','#00BA49','#00BAA6']
    # # labels_RR = ['','No Swarm','Waypoint\nControl','User','Shared','Autonomous']
    # labels_RR = ['','Baseline','1-Robot\nCommands','3-Robot\nCommands','Shared\nControl','Fully\nAutonomous']


    for met in range(len(metrics)):
        if 'RR_Zscore' == metrics[met]:
            RR_list = data_list[met]

    data_low = RR_list[:5]
    data_high = RR_list[5:]
    if combine_environments:
        data = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
        num_environments = 1
    else:
        num_environments = 2



    for plot_num in range(num_environments):
        if not combine_environments:
            if plot_num==0:
                data = data_low
            else:
                data = data_high

        # Add stastical significant basedon ANOVA #################################
        title = '\'RR\' Interval'
        for i in range(len(data)):
            data[i] = -1*data[i]
        if only_experts:
            if combine_environments:
                sig_matrix = np.array([
                                [0, 1, 0.025],  # none - waypoint
                                [1, 3, 0.032],  # waypoint - sharedergodic
                                [1, 4, 0.008]])  # waypoint - autoergodic
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                    [0, 1, 0.025]])  # none - waypoint
                else:
                    sig_matrix = np.array([])
        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([])
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([])
        else:
            sig_matrix = np.array([])

        # Create a plot ##########################################################
        ylabel = 'Within-Subject Z-score\n(lower numbers = more cognitive availability)'
        if combine_environments and plot_num==0:
        # if only_experts and (not combine_environments) and plot_num==0:
            ylabel = 'Within-Subject Z-score'
        xlabel = ''
        fig, ax = plt.subplots(figsize=figure_size,dpi=300)
        upper_data_bound = make_scatter(fig, ax, data, title, xlabel, ylabel, labels5, colors5)

        # Add stastical signicant marking #########################################
        add_stats(upper_data_bound,sig_matrix,ax,spread_factor=20,type='bar')

        # Add *Ergodic* label and arrow on the bottom of the plot #################
        # fig.subplots_adjust(bottom=0.18)  # asjust white spacing on the bottom
        fig.subplots_adjust(bottom=0.23,left=.18)  # asjust white spacing on the bottom
        if only_experts:
            if not combine_environments:
                ax.set_ylim([-.8,1.25])
        # x1 = [2.75, 0]  # start of the arrow
        # x2 = [5.25, 0]  # end of the arrow
        # text_buffer = .1
        # name = ['Coverage Control', '']  # single label due to combining env. complexity
        #
        # add_labels(ax, x1, x2, y, name, text_buffer)

        # Saving the plots ########################################################
        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'RR/RR_experts.pdf')
                fig.savefig(file_plot + 'RR/RR_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'RR/RR_experts_lowD.pdf')
                    fig.savefig(file_plot + 'RR/RR_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'RR/RR_experts_highD.pdf')
                    fig.savefig(file_plot + 'RR/RR_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'RR/RR_novices.pdf')
                fig.savefig(file_plot + 'RR/RR_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'RR/RR_novices_lowD.pdf')
                    fig.savefig(file_plot + 'RR/RR_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'RR/RR_novices_highD.pdf')
                    fig.savefig(file_plot + 'RR/RR_novices_highD.png')
        else:
            fig.savefig(file_plot + 'RR/RR.pdf')
            fig.savefig(file_plot + 'RR/RR.png')

def fnct_plot_MDP(metrics,data_list,only_experts,only_novices,combine_environments,file_plot,labels5, colors5, alphas5,figure_size):

    for met in range(len(metrics)):
        if 'MDP_r' == metrics[met]:
            MDP_list = data_list[met]

    data_low = MDP_list[:5]
    data_high = MDP_list[5:]
    if combine_environments:
        data = []
        for i in range(len(data_low)):
            data_combined = np.concatenate((data_low[i], data_high[i]), axis=0)
            data.append(data_combined)
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):
        if not combine_environments:
            if plot_num==0:
                data = data_low
            else:
                data = data_high


        title = 'Simulated Replica of Environment\nMirrors Task Performance in Virtual Reality'
        if only_experts:
            if combine_environments:
                sig_matrix = np.array([])
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                            [3,4,0.0070],  # shared - auto
                                            [2,4,0.0127],  # direct - auto
                                            ])
                else:
                    sig_matrix = np.array([])
        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([])
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([
                                            [3,4,0.0354],  # shared - auto
                                            ])
        else:
            sig_matrix = np.array([])

        # Create a plot ##########################################################
        ylabel = 'Cumulative Regret With\nAll Task Information'
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
        if combine_environments:
            ax.set_ylim([3,7])
        # Saving the plots ########################################################
        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'MDP/MDP_experts.pdf')
                fig.savefig(file_plot + 'MDP/MDP_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'MDP/MDP_experts_lowD.pdf')
                    fig.savefig(file_plot + 'MDP/MDP_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'MDP/MDP_experts_highD.pdf')
                    fig.savefig(file_plot + 'MDP/MDP_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'MDP/MDP_novices.pdf')
                fig.savefig(file_plot + 'MDP/MDP_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'MDP/MDP_novices_lowD.pdf')
                    fig.savefig(file_plot + 'MDP/MDP_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'MDP/MDP_novices_highD.pdf')
                    fig.savefig(file_plot + 'MDP/MDP_novices_highD.png')
        else:
            fig.savefig(file_plot + 'MDP/MDP.pdf')
            fig.savefig(file_plot + 'MDP/MDP.png')

def fnct_plot_POMDP_d(only_experts,only_novices,combine_environments,file_plot,file_POMDP_d,file_pomdo_d_new,figure_size_input,colors5):
    df_regret = pd.read_csv(file_POMDP_d)
    df_regret["Percent_regret"] = df_regret["Regret"]/df_regret["Maximum_Regret"]
    df_regret["Max_Impact"] = df_regret["Maximum_Regret"]
    df_regret["Max_group"] = df_regret["Maximum_Regret"]
    n_group = np.zeros(3)
    for i in range(df_regret.shape[0]):
        if df_regret["Max_Impact"].at[i]<=1:
            n_group[0] += 1
            df_regret["Max_Impact"].at[i] = "0-1"
        elif df_regret["Max_Impact"].at[i]<=2:
            n_group[1] += 1
            df_regret["Max_Impact"].at[i] = "1-2"
        # elif df_regret["Max_Impact"].at[i]<=2:
        #     df_regret["Max_Impact"].at[i] = "2-3"
        #     n_group[2] += 1
        else:
            df_regret["Max_Impact"].at[i] = "3+"

    # print(n_group)
    df_regret.to_csv(file_pomdo_d_new, encoding='utf-8', index=False)
    df_regret = df_regret[(df_regret["Max_Impact"]!="3+")]
    # df_regret = df_regret[(df_regret["Max_Impact"]!="3+") & (df_regret["Max_Impact"]!="2-3")]
    # df_regret = df_regret[df_regret["Max_Impact"]=='.6-1']

    if combine_environments:
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):

        # Set up figure parameters
        labels5 = ['Baseline','Waypoint','User','Shared\nControl','Fully\nAutonomous']
        title = 'Human Performance'
        xlabel = ''
        ylabel = 'Percent Regret Compared to Optimal Agent'

        if only_experts:
            df = df_regret[df_regret["Expertise"]=="expert"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        elif only_novices:
            df = df_regret[df_regret["Expertise"]=="novice"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        else:
            df = df_regret

        for plot_style in range(2):
            if plot_style==0:
                continue # don't make combined plots
            if plot_style==1:
                fig, ax = plt.subplots(figsize=(7, 3.25),dpi=300)
            else:
                fig, ax = plt.subplots(figsize=figure_size_input,dpi=300)

            if plot_style==1:
                hue_order = ["0-1", "1-2"]
                plots = sns.barplot(data=df, x="Control", y="Percent_regret", hue="Max_Impact", hue_order=hue_order,
                            ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.1)
                            # ax=ax, errcolor='black', ci=68/2, errwidth = 1, capsize=.1)
            else:
                plots = sns.barplot(data=df, x="Control", y="Percent_regret", palette = colors5, saturation = 1,
                            ax=ax, errcolor='black', ci=68, errwidth = 1, capsize=.1)

            ax.set_ylim(bottom=0.25)

            if plot_style==1:
                # Iterating over the bars one-by-one
                colors15 = [#'#BA4900','#BA0071','#0071BA','#00BA49','#3c2692ff',
                '#ff8333ff','#ff33afff','#33afffff','#33ff83ff','#765dd5ff',
                '#ffc199ff','#ff99d7ff','#99d7ffff','#99ffc1ff','#baaeeaff']
                # i = 0
                # print(len(plots.patches),plots.patches)
                for i in range(10):
                    # print(i)
                    bar = plots.patches[i]
                    if i<5:
                        text = 'L'
                    elif i<10:
                        text = 'H'
                    # else:
                    #     text = 'H'
                    # for bar in plots.patches:

                  # Using Matplotlib's annotate function and
                  # passing the coordinates where the annotation shall be done
                  # x-coordinate: bar.get_x() + bar.get_width() / 2
                  # y-coordinate: bar.get_height()
                  # free space to be left to make graph pleasing: (0, 8)
                  # ha and va stand for the horizontal and vertical alignment
                    y_lim = ax.get_ylim()
                    plots.annotate(text,
                               (bar.get_x() + bar.get_width() / 2,
                                y_lim[0]), ha='center', va='center',
                               size=15, xytext=(0, 8),
                               textcoords='offset points', fontname="sans-serif", fontsize=9)
                    bar.set_color(matplotlib.colors.to_rgb(colors15[i]))

                    # i += 1

                i = 0
                bars_to_remove = []
                control = ['none', 'waypoint', 'directergodic', 'sharedergodic', 'autoergodic']
                for hue in hue_order:
                    for con in control:
                        df_temp = df[(df["Control"]==con) & (df["Max_Impact"]==hue)]
                        print(con,hue,len(df_temp))
                        # bar = plots.patches[i]
                        if len(df_temp)<20:
                            bars_to_remove.append(i)
                            # bar.remove()
                        i += 1

                # for i in range(14,0,-1):
                for i in range(9,0,-1):
                    bar = plots.patches[i]
                    for ii in bars_to_remove:
                        if ii==i:
                            bar.remove()
                # sns.barplot(data=df_regret, x="Control", y="Percent_regret", ax=ax, errcolor='black')
                # sns.stripplot(data=df_regret, x="Control", y="Percent_regret", hue="Max_Impact", ax=ax, size=3) #jitter=.2,
                # sns.swarmplot(data=df_regret, x="Control", y="Percent_regret", hue="Max_Impact", color='black', ax=ax, size=1.75) #jitter=.2,

                # x-ticks x-axis
                # if only_experts and (not combine_environments) and plot_num==0:
                ax.get_legend().remove()

            # Add titles and labels
            plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
            plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
            plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
            ax.set_xticklabels(labels5, fontname="sans-serif", fontsize=9)
            for label in (ax.get_yticklabels()):
                label.set_fontsize(8)
            for tick in ax.get_xticklabels():
                tick.set_rotation(30)

            # if only_experts and (not combine_environments) and plot_num==0:
            # labels = ['L: 0-0.67','M: 0.67-1.33','H: 1.33-2']
            if plot_style==1:
                labels = ['L: 0-1','H: 1-2']
                fig.legend(labels,loc="center right")
                fig.subplots_adjust(right=.75)  # adjust white spacing

            ax.grid(True, linestyle='-', which='major', axis='y', color='lightgrey',
                       alpha=0.5)
            ax.set_axisbelow(True)
            # ax.set_ylim([.25,.8])

            # Add stastical signicant marking #########################################
            if only_experts:
                if combine_environments:
                    sig_matrix = np.array([
                                            [0,1,0.00343],  # none - waypoint
                                            [3,4,0.02729],  # shared - auto
                                            [1,2,0.0009],  # wp - direct
                                            [1,3,0.0009],  # wp - shared
                                            [1,4,0.0009],  # wp - auto
                                            ])
                else:
                    if plot_num==0:
                        sig_matrix = np.array([
                                                [0,1,0.0009],  # wp - none
                                                [1,2,0.0009],  # wp - direct
                                                [1,3,0.06426],  # wp - shared
                                                [1,4,0.0009],  # wp - auto
                                                [0,2,0.00747],  # none - direct
                                                [0,3,0.0009],  # none - shared
                                                [0,4,0.0009],  # none - auto
                                                ])
                    else:
                        sig_matrix = np.array([
                                                [3,4,0.02141],  # shared - auto
                                                [0,1,0.00274],  # wp - none
                                                [1,2,0.0009],  # wp - direct
                                                [1,3,0.0009],  # wp - shared
                                                [1,4,0.0009],  # wp - auto
                                                [0,4,0.12461],  # none - auto
                                                ])
            elif only_novices:
                if combine_environments:
                    sig_matrix = np.array([])
                else:
                    if plot_num==0:
                        sig_matrix = np.array([])
                    else:
                        sig_matrix = np.array([])

            else:
                sig_matrix = np.array([
                                        [1,4,0.04687],  # wp - auto
                                        [0,2,0.00653],  # none - direct
                                        [0,3,0.0009],  # shared - none
                                        [0,4,0.0009],  # none - auto
                                        ])
            # upper_data_bound = [.35,.5,.5,.465,.43]
            upper_data_bound = [.44,.47,.44,.42,.4]
            ax.set_ylim([.25,.65])
            if only_experts:
                if combine_environments:
                    ax.set_ylim([.25,.56])
                else:
                    ax.set_ylim([.25,.65])
                    if plot_style==0:
                        ax.set_ylim([.25,.6])
                        upper_data_bound = []
                        control_options = ['none','waypoint','directergodic','sharedergodic','autoergodic']
                        for i in range(len(control_options)):
                            df_temp = df[df['Control']==control_options[i]]
                            ub = df_temp['Percent_regret'].mean() + df_temp['Percent_regret'].sem()
                            upper_data_bound.append(ub)
                        # upper_data_bound = [.39,.42,.39,.37,.35]
            if only_novices:
                ax.set_ylim([.25,.55])
                # if combine_environments:
                # else:
                #     ax.set_ylim([.25,.75])
            add_stats(upper_data_bound,sig_matrix,ax,spread_factor=20,type='bar')

            fig.subplots_adjust(bottom=0.23,left=.1)  # asjust white spacing

            # Saving the plots ########################################################
            # if plot_style==1:
            if only_experts:
                if combine_environments:
                    fig.savefig(file_plot + 'Decisions/pomdo_d_experts'+str(plot_style)+'.pdf')
                    fig.savefig(file_plot + 'Decisions/pomdo_d_experts'+str(plot_style)+'.png')
                else:
                    if plot_num==0:
                        fig.savefig(file_plot + 'Decisions/pomdo_d_experts_lowD'+str(plot_style)+'.pdf')
                        fig.savefig(file_plot + 'Decisions/pomdo_d_experts_lowD'+str(plot_style)+'.png')
                    else:
                        fig.savefig(file_plot + 'Decisions/pomdo_d_experts_highD'+str(plot_style)+'.pdf')
                        fig.savefig(file_plot + 'Decisions/pomdo_d_experts_highD'+str(plot_style)+'.png')
            elif only_novices:
                if combine_environments:
                    fig.savefig(file_plot + 'Decisions/pomdo_d_novices'+str(plot_style)+'.pdf')
                    fig.savefig(file_plot + 'Decisions/pomdo_d_novices'+str(plot_style)+'.png')
                else:
                    if plot_num==0:
                        fig.savefig(file_plot + 'Decisions/pomdo_d_novices_lowD'+str(plot_style)+'.pdf')
                        fig.savefig(file_plot + 'Decisions/pomdo_d_novices_lowD'+str(plot_style)+'.png')
                    else:
                        fig.savefig(file_plot + 'Decisions/pomdo_d_novices_highD'+str(plot_style)+'.pdf')
                        fig.savefig(file_plot + 'Decisions/pomdo_d_novices_highD'+str(plot_style)+'.png')
            else:
                fig.savefig(file_plot + 'Decisions/pomdo_d'+str(plot_style)+'.pdf')
                fig.savefig(file_plot + 'Decisions/pomdo_d'+str(plot_style)+'.png')

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

def fnct_plot_N_obs(only_experts,only_novices,combine_environments,file_plot,labels4,colors4,alphas4,figure_size):

    df_raw_formatted = pd.read_csv('raw_data_formatted/raw_data_formatted.csv')
    df_N_obs = df_raw_formatted[(df_raw_formatted['Include_N_obs']==True) &
                                (df_raw_formatted['Control']!='none')]


    if combine_environments:
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):
        # Set up figure parameters
        title = 'Robotic Performance\nat Detecting People'
        xlabel = ''
        ylabel = 'Number of People Found\nDuring 5min Trial'


        if only_experts:
            df = df_N_obs[df_N_obs["Expertise"]=="expert"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        elif only_novices:
            df = df_N_obs[df_N_obs["Expertise"]=="novice"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        else:
            df = df_N_obs
        # Plot data
        fig, ax = plt.subplots(figsize=figure_size,dpi=300)
        sns.barplot(data=df_N_obs, x="Control", y="N_obs", palette = colors4, saturation = alphas4,
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
        if only_experts:
            if combine_environments:
                sig_matrix = np.array([
                                        [0,1,0.0009],  # wp - direct
                                        [1,2,0.0009],  # direct - shared
                                        [2,3,0.017],  # shared - auto
                                        [0,2,0.0009],  # wp - shared
                                        [1,3,0.0009],  # direct - auto
                                        [0,3,0.0009]  # wp - auto
                                        ])
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                            [0,1,0.0009],  # wp - direct
                                            [1,2,0.0204],  # direct - shared
                                            [2,3,0.0043],  # shared - auto
                                            [0,2,0.0009],  # wp - shared
                                            [1,3,0.0009],  # direct - auto
                                            [0,3,0.0009]  # wp - auto
                                            ])
                else:
                    sig_matrix = np.array([
                                            [0,1,0.0009],  # wp - direct
                                            [1,2,0.0009],  # direct - shared
                                            # [2,3,0.017],  # shared - auto
                                            [0,2,0.0009],  # wp - shared
                                            [1,3,0.0009],  # direct - auto
                                            [0,3,0.0009]  # wp - auto
                                            ])
        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([
                                        [0,1,0.003],  # wp - direct
                                        [1,2,0.0078],  # direct - shared
                                        # [2,3,0.017],  # shared - auto
                                        [0,2,0.0009],  # wp - shared
                                        [1,3,0.0017],  # direct - auto
                                        [0,3,0.0009]  # wp - auto
                                        ])
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                            # [0,1,0.0009],  # wp - direct
                                            [1,2,0.095],  # direct - shared
                                            # [2,3,0.017],  # shared - auto
                                            [0,2,0.0009],  # wp - shared
                                            # [1,3,0.0009],  # direct - auto
                                            [0,3,0.0009]  # wp - auto
                                            ])
                else:
                    sig_matrix = np.array([
                                            [0,1,0.0655],  # wp - direct
                                            # [1,2,0.0078],  # direct - shared
                                            [2,3,0.033],  # shared - auto
                                            [0,2,0.003],  # wp - shared
                                            [1,3,0.015],  # direct - auto
                                            [0,3,0.0009]  # wp - auto
                                            ])
        else:
            sig_matrix = np.array([
                                    [0,1,0.0009],  # wp - direct
                                    [1,2,0.0009],  # direct - shared
                                    [2,3,0.014],  # shared - auto
                                    [0,2,0.0009],  # wp - shared
                                    [1,3,0.0009],  # direct - auto
                                    [0,3,0.0009]  # wp - auto
                                    ])
        upper_data_bound = []
        control_options = ['waypoint','directergodic','sharedergodic','autoergodic']
        for i in range(len(control_options)):
            df_temp = df_N_obs[df_N_obs['Control']==control_options[i]]
            upper_data_bound.append(df_temp['N_obs'].mean())
        add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
        # ax.set_ylim([.25,.64])

        # fig.subplots_adjust(bottom=0.15)  # asjust white spacing
        fig.subplots_adjust(bottom=0.25,left=.3,right=.85)  # asjust white spacing

        # Saving the plots ########################################################
        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'N_found/n_observations_experts.pdf')
                fig.savefig(file_plot + 'N_found/n_observations_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'N_found/n_observations_experts_lowD.pdf')
                    fig.savefig(file_plot + 'N_found/n_observations_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'N_found/n_observations_experts_highD.pdf')
                    fig.savefig(file_plot + 'N_found/n_observations_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'N_found/n_observations_novices.pdf')
                fig.savefig(file_plot + 'N_found/n_observations_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'N_found/n_observations_novices_lowD.pdf')
                    fig.savefig(file_plot + 'N_found/n_observations_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'N_found/n_observations_novices_highD.pdf')
                    fig.savefig(file_plot + 'N_found/n_observations_novices_highD.png')
        else:
            fig.savefig(file_plot + 'N_found/n_observations.pdf')
            fig.savefig(file_plot + 'N_found/n_observations.png')

def fnct_plot_POMDP_obs(df_initial,metric,only_experts,only_novices,combine_environments,file_plot,labels4,colors4,alphas4,figure_size):

    # df = pd.read_csv('raw_data/raw_data_POMDP_obs_i_6a_10000s.csv')
    # df = pd.read_csv('raw_data/raw_data_POMDP_obs_cum_6a_10000s.csv')
    # metric = "P_switch"
    # metric = "P_switch_cum"

    # Set up figure parameters
    if metric == "P_switch":
        title = 'Utility of Each Robot\nObservation to Human'
        xlabel = ''
        ylabel = 'Probability Optimal Path Changes\nDue to New Observation'
    elif metric == "P_switch_cum":
        title = 'Utility of Collective Robotic\nBehavior to Human'
        xlabel = ''
        ylabel = 'Probability Optimal Path Changes\nDue to New Observations'
    else:
        title = metric
        xlabel = ''
        ylabel = ''
    sig_matrix = np.array([])

    if combine_environments:
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):

        if only_experts:
            df = df_initial[df_initial["Expertise"]=="expert"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        elif only_novices:
            df = df_initial[df_initial["Expertise"]=="novice"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        else:
            df = df_initial


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
        if metric == "P_switch":
            if only_experts:
                if combine_environments:
                    sig_matrix = np.array([])
                else:
                    if plot_num==0:
                        sig_matrix = np.array([
                                    [0, 3, 0.0109], # waypoint - auto
                                    [0,0,1]])
                    else:
                        sig_matrix = np.array([])
            elif only_novices:
                if combine_environments:
                    sig_matrix = np.array([
                                [1, 3, 0.0009], # direct - auto
                                [2, 3, 0.0073], # shared - auto
                                [0,0,1]])
                else:
                    if plot_num==0:
                        sig_matrix = np.array([])
                    else:
                        sig_matrix = np.array([
                                    [1, 3, 0.0013], # direct - auto
                                    [2, 3, 0.008], # shared - auto
                                    ])
            else:
                sig_matrix = np.array([
                            [1, 3, 0.0303], # direct - auto
                            [0,0,1]])

            upper_data_bound = []
            control_options = ['waypoint','directergodic','sharedergodic','autoergodic']
            for i in range(len(control_options)):
                df_temp = df[df['Control']==control_options[i]]
                upper_data_bound.append(df_temp[metric].mean())
            add_stats(upper_data_bound,sig_matrix,ax,spread_factor=20,type='bar')
            ax.set_ylim(bottom=.3)
        elif metric == "P_switch_cum":
            if only_experts:
                if combine_environments:
                    sig_matrix = np.array([])
                else:
                    if plot_num==0:
                        sig_matrix = np.array([])
                    else:
                        sig_matrix = np.array([])
            elif only_novices:
                if combine_environments:
                    sig_matrix = np.array([])
                else:
                    if plot_num==0:
                        sig_matrix = np.array([])
                    else:
                        sig_matrix = np.array([])
            else:
                sig_matrix = np.array([])

            upper_data_bound = []
            control_options = ['waypoint','directergodic','sharedergodic','autoergodic']
            for i in range(len(control_options)):
                df_temp = df[df['Control']==control_options[i]]
                upper_data_bound.append(df_temp[metric].mean())
            add_stats(upper_data_bound,sig_matrix,ax,spread_factor=22,type='bar')
            ax.set_ylim(bottom=.3)

        fig.subplots_adjust(bottom=0.25,left=.3,right=.85)  # asjust white spacing

        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'Observations/' + metric + '_experts.pdf')
                fig.savefig(file_plot + 'Observations/' + metric + '_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Observations/' + metric + '_experts_lowD.pdf')
                    fig.savefig(file_plot + 'Observations/' + metric + '_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'Observations/' + metric + '_experts_highD.pdf')
                    fig.savefig(file_plot + 'Observations/' + metric + '_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'Observations/' + metric + '_novices.pdf')
                fig.savefig(file_plot + 'Observations/' + metric + '_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Observations/' + metric + '_novices_lowD.pdf')
                    fig.savefig(file_plot + 'Observations/' + metric + '_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'Observations/' + metric + '_novices_highD.pdf')
                    fig.savefig(file_plot + 'Observations/' + metric + '_novices_highD.png')
        else:
            fig.savefig(file_plot + 'Observations/' + metric + '.pdf')
            fig.savefig(file_plot + 'Observations/' + metric + '.png')

def fnct_plot_performance(only_experts,only_novices,combine_environments,file_plot,labels5,colors5,alphas5,figure_size):

    df_raw_formatted = pd.read_csv('raw_data_formatted/raw_data_formatted.csv')
    df_s = df_raw_formatted[(df_raw_formatted['Include_Score']==True)]
    df_s['Lives*3'] = df_s['Lives'].values * 3

    # Set up figure parameters
    title = 'Game Performance'
    xlabel = ''
    ylabel = 'Final Game Score'

    if combine_environments:
        num_environments = 1
    else:
        num_environments = 2

    for plot_num in range(num_environments):

        if only_experts:
            df = df_s[df_s["Expertise"]=="expert"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        elif only_novices:
            df = df_s[df_s["Expertise"]=="novice"]
            if not combine_environments:
                if plot_num==0:
                    df = df[df["Density"]=="low"]
                else:
                    df = df[df["Density"]=="high"]
        else:
            df = df_s

        if only_experts:
            if combine_environments:
                sig_matrix = np.array([])
            else:
                if plot_num==0:
                    sig_matrix = np.array([
                                [3, 4, 0.0275], # shared - auto
                                [0,0,1]])
                else:
                    sig_matrix = np.array([])
        elif only_novices:
            if combine_environments:
                sig_matrix = np.array([
                            [1, 4, 0.0427], # waypoint - auto
                            [0,0,1]])
            else:
                if plot_num==0:
                    sig_matrix = np.array([])
                else:
                    sig_matrix = np.array([
                                [1, 4, 0.0186], # waypoint - auto
                                [0,0,1]])
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
            val = df_temp['Score'].mean() + df_temp['Score'].sem()
            upper_data_bound.append(val)
        add_stats(upper_data_bound,sig_matrix,ax,spread_factor=90,type='bar')
        # ax.set_ylim([.25,.64])

        fig.subplots_adjust(bottom=0.15)  # asjust white spacing

        # Saving the plots ########################################################
        ax.set_ylim(bottom=18, top=27)
        if only_experts:
            if combine_environments:
                fig.savefig(file_plot + 'Score/score_bar_experts.pdf')
                fig.savefig(file_plot + 'Score/score_bar_experts.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Score/score_bar_experts_lowD.pdf')
                    fig.savefig(file_plot + 'Score/score_bar_experts_lowD.png')
                else:
                    fig.savefig(file_plot + 'Score/score_bar_experts_highD.pdf')
                    fig.savefig(file_plot + 'Score/score_bar_experts_highD.png')
        elif only_novices:
            if combine_environments:
                fig.savefig(file_plot + 'Score/score_bar_novices.pdf')
                fig.savefig(file_plot + 'Score/score_bar_novices.png')
            else:
                if plot_num==0:
                    fig.savefig(file_plot + 'Score/score_bar_novices_lowD.pdf')
                    fig.savefig(file_plot + 'Score/score_bar_novices_lowD.png')
                else:
                    fig.savefig(file_plot + 'Score/score_bar_novices_highD.pdf')
                    fig.savefig(file_plot + 'Score/score_bar_novices_highD.png')
        else:
            fig.savefig(file_plot + 'Score/score_bar.pdf')
            fig.savefig(file_plot + 'Score/score_bar.png')
