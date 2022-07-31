import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def add_stats(data, sig_matrix, ax, spread_factor=30, type='boxplot'):
    """
    Add asterisks to the plot to indicate significance; makes it so the bars and
     *s don't overlap
    Inputs:
        data - depending on the type, this is used to ensure the significance lines
            do not overlap with the data. For boxplot, format it in a list of numpy arrays.
            For bar, it is a list of the max value on the plot
        sig_matrix - numpy array with a row for each significant pair. The columns
            are as follows [factor1(0-N) factor2(0-N) p-value]
        ax - the plot axes to place the significance lines
        spread_factor - if this number is greater, the significance bars are closer together, and if
                    it is smaller it is further apart
        type - indicated what type of plot you are adding to-- this is used to make
            sure the lines do not overlap with the plot
    Outputs: n/a
    """

    # Checks to make sure the plot exists
    if sig_matrix.shape[0] == 0:
        return

    # Matrix for storing where lines have been placed above every column
    sig_fill = np.zeros((sig_matrix.shape[0], len(data)))

    # Set variables for spacing lines
    [ymin, ymax] = ax.get_ylim()
    min_dist = (ymax-ymin)/spread_factor  # indicates the minimum distance from the data
    tol = min_dist  # indicates the minimum spacing between two lines

    # Iterates through each significant p-value (row in sig_matrix)
    for i in range(sig_matrix.shape[0]):


        # obtain values from s/ig_matrix
        fac1 = int(sig_matrix[i, 0])
        fac2 = int(sig_matrix[i, 1])
        pval = sig_matrix[i, 2]

        if pval < 0.05:
            # determine whether to step up or down when iterating
            if fac1 > fac2:
                temp = fac2
                fac2 = fac1
                fac1 = temp
            step = 1

            # Find the minimum height of the line so that is doesn't overlap
            # with the data
            min_liney = 0
            for ii in range(fac1, fac2+1, step):
                if type == 'boxplot':
                    ii_max = np.max(data[ii])
                elif type == 'bar':
                    ii_max = data[ii]
                if ii_max > min_liney:
                    min_liney = ii_max
            min_liney += min_dist + tol
            # print('starting y:',min_liney)

            # increase the location of y_topline until one is suitable
            notfound = True
            y_topline = min_liney
            if i == 0:
                notfound = False
            while notfound:
                notfound = False
                for ii in range(fac1, fac2+1):  # iterate though overlapping trial conditions
                    for j in range(i):  # iterate through all of the line locations
                        if abs(y_topline-sig_fill[j, ii]) + (tol/100) < tol:  # This line location is not suitable
                            y_topline += tol
                            notfound = True
                            # print(j, ii, y_topline, sig_fill[j, ii],tol)
                            break


            # fill array with chosen locations
            # print('final y: ',y_topline)
            for ii in range(fac1, fac2+1):
                sig_fill[i, ii] = y_topline
            # print('sig_fill: ',sig_fill)

            # Plot the line
            # get points for line notches
            if type == 'boxplot':
                fac1_max = np.max(data[fac1])
                fac2_max = np.max(data[fac2])
                fac1 += 1
                fac2 += 1
            elif type == 'bar':
                fac1_max = data[fac1]
                fac2_max = data[fac2]
            # print(fac1,fac2)
            # print(fac1_max,fac2_max)
            ax.plot([fac1, fac1, fac2, fac2],
            # ax.plot([fac1 + 1, fac1 + 1, fac2 + 1, fac2 + 1],
                    [fac1_max+min_dist, y_topline, y_topline, fac2_max+min_dist],
                    '-k', markersize=5)

            # xloc = fac1 + 1 + ((fac2-fac1)/2.0)
            xloc = fac1 + ((fac2-fac1)/2.0)
            yloc = y_topline - (tol/6)

            if pval < 0.001:
                ax.text(xloc, yloc, '***', horizontalalignment='center', fontsize=8, fontweight='bold')
            elif pval < 0.01:
                ax.text(xloc, yloc, '**', horizontalalignment='center', fontsize=8, fontweight='bold')
            elif pval < 0.05:
                ax.text(xloc, yloc, '*', horizontalalignment='center', fontsize=8, fontweight='bold')

    return

def add_labels(ax, x1, x2, y, name, text_buffer):
    """
    Adds arrows and labels to the bottom of plot by providing the locations in
    plot coordinates
    Inputs:
        ax - indicated plot axis
        x1 - list of x-values for the starting point of the arrows
        x2 - list of x-values for the ending point of the arrows
        y - float for the y-location of the arrow
        name - list of names for the arrows
        buffer - the size of the buffer between the arrow and text
    Outputs: n/a
    """

    text_height = y-text_buffer
    for i in range(len(x1)):
        text_x = x1[i] + (x2[i]-x1[i])/2
        ax.annotate('', xy=(x1[i], y), xytext=(x2[i], y),  # draws an arrow from one set of coordinates to the other
                    arrowprops=dict(arrowstyle='<|-|>', facecolor='black'),  # sets style of arrow and colour
                    annotation_clip=False)  # This enables the arrow to be outside of the plot
        ax.text(text_x, text_height, name[i],
                horizontalalignment='center', fontname="sans-serif", fontsize=9)

    # l2_x1 = [-.5,4.5]
    # l2_x2 = [4.5,9.5]
    # l2_y = 14
    # l2_name = ['Low Density','High Density']
    #
    # text_height = l2_y-.6
    # for i in range(len(l1_x1)):
    #     text_x = l2_x1[i] + (l2_x2[i]-l2_x1[i])/2
    #     ax.annotate('', xy=(l2_x1[i],l2_y),xytext=(l2_x2[i],l2_y),                     #draws an arrow from one set of coordinates to the other
    #                 arrowprops=dict(arrowstyle='<|-|>',facecolor='black'),   #sets style of arrow and colour
    #                 annotation_clip=False)                               #This enables the arrow to be outside of the plot
    #     ax.text(text_x, text_height,l2_name[i],horizontalalignment='center', fontname="sans-serif", fontsize=9)


    # ax.annotate('', xy=(6.5,a1_y),xytext=(9.5,a1_y),                     #draws an arrow from one set of coordinates to the other
    #             arrowprops=dict(arrowstyle='<|-|>',facecolor='black'),   #sets style of arrow and colour
    #             annotation_clip=False)                               #This enables the arrow to be outside of the plot
    # ax.text(.83, text_height,'Ergodic',horizontalalignment='center', fontname="sans-serif", fontsize=9)
    # arrow_height = 14.25
    # text_height = -.28
    # ax.annotate('', xy=(-.5,arrow_height),xytext=(4.5,arrow_height),                     #draws an arrow from one set of coordinates to the other
    #             arrowprops=dict(arrowstyle='<|-|>',facecolor='black'),   #sets style of arrow and colour
    #             annotation_clip=False)                               #This enables the arrow to be outside of the plot
    # ax.text(.26, text_height,'Low Density',horizontalalignment='center', fontname="sans-serif", fontsize=9)
    # ax.annotate('', xy=(4.5,arrow_height),xytext=(9.5,arrow_height),                     #draws an arrow from one set of coordinates to the other
    #             arrowprops=dict(arrowstyle='<|-|>',facecolor='black'),   #sets style of arrow and colour
    #             annotation_clip=False)                               #This enables the arrow to be outside of the plot
    # ax.text(.75, text_height,'High Density',horizontalalignment='center', fontname="sans-serif", fontsize=9)

    return

def make_stackedbar(data1, data2, title, xlabel, ylabel, labels, colors, alphas, figure_size):
    """
    Creates and formats a stacked bar plot
    Inputs:
        data1 - The original data for the lower bars formatted it in a list of numpy arrays
        data2 - The original data for the upper bars formatted it in a list of numpy arrays
        title/xlabel/ylabel - strings for each label
        labels - list of strings corresponding to each dataset
        colors - list of strings corresponding to the desired color for each dataset
        alphas - list of floats from 0 to 1 corresponding to the desired
            tranparency for each list item in both datasets
        figure_size - tuple for figure size
    Outputs: the fig and ax labels for the figure and the upper_data_bound for
        each condition on the plot for adding significant asterisks
    """
    fig, ax = plt.subplots(figsize=figure_size, dpi=300)

    n1 = len(data1)
    n2 = len(data2)
    if n1 != n2:
        print('Data not of the same length')

    # Defines x-axis values
    ind = np.arange(n1)

    width = 0.5  # width of each plot

    # plot data bars sequentially
    upper_data_bound = []  # store the upper bar for stat testing
    for i in range(n1):
        data_2 = data1[i] + data2[i]
        data1_mean = np.mean(data1[i])
        data2_mean = np.mean(data2[i])
        data2_std = np.std(data_2)/np.sqrt(data_2.shape[0])
        upper_data_bound.append(data1_mean+data2_mean+data2_std)

        if i == 1:
            p1 = ax.bar(ind[i], data1_mean, width,# yerr=data1_std,
                        ecolor='black', capsize=5, color=colors[i],
                        alpha=alphas[i],edgecolor='black')
            p2 = ax.bar(ind[i], data2_mean, width, bottom=data1_mean,
                        yerr=data2_std, ecolor='black', capsize=5,
                        color=colors[i], alpha=alphas[i], hatch='..',edgecolor='black')
        else:
            ax.bar(ind[i], data1_mean, width,# yerr=data1_std,
                   ecolor='black', capsize=5, color=colors[i],
                   alpha=alphas[i],edgecolor='black')
            ax.bar(ind[i], data2_mean, width, bottom=data1_mean, yerr=data2_std,
                   ecolor='black', capsize=5, color=colors[i],
                   alpha=alphas[i], hatch='..',edgecolor='black')

    # # place grid in back
    # ax.grid(True, linestyle='-', which='major', axis='y',
    #         color='lightgrey', alpha=0.5)
    # ax.set_axisbelow(True)

    # Add titles and labels
    plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
    plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
    plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    for label in (ax.get_yticklabels()):
        label.set_fontsize(8)

    # figure legend
    L = fig.legend([p1[0], p2[0]], ['Lives', 'Treasures'], ncol=1, fontsize=9,
                   loc='upper center', bbox_to_anchor=(0.46, .79, 0.1, 0.1))
    plt.setp(L.texts, family='sans-serif')

    # x-ticks x-axis
    plt.xticks(ind, labels, fontname="sans-serif", fontsize=9)
    for tick in ax.get_xticklabels():
        tick.set_rotation(0)

    return [fig, ax, upper_data_bound]

def make_scatter(fig, ax, data, title, xlabel, ylabel, labels, colors):
    """
    Creates and formats a scatter plot
    Inputs:
        fig, ax - created using plt.subplots(figsize=figure_size, dpi=300)
        data - The original data for each point formatted it in a list of numpy arrays
        title/xlabel/ylabel - strings for each label
        labels - list of strings corresponding to each dataset
        marker_type - list of strings corresponding to each markertype
        colors - list of strings corresponding to the desired color for each dataset
        figure_size - tuple for figure size
    Outputs: the fig and ax labels for the figure and the upper_data_bound for
        each condition on the plot for adding significant asterisks
    """
    # fig, ax = plt.subplots(figsize=figure_size, dpi=300)
    n = len(data)

    # Defines x-axis values
    ind = np.arange(n)

    # plot data bars sequentially
    upper_data_bound = []  # store the upper bar for stat testing
    for i in range(n):
        data_mean = np.mean(data[i])
        data_std = np.std(data[i])/np.sqrt(data[i].shape[0])

        upper_data_bound.append(data_mean+data_std)

        p1 = ax.errorbar(ind[i], data_mean, yerr=data_std, markersize=9, markeredgecolor='black',
                    markeredgewidth = 1,
                    ecolor='black', capsize=5, color=colors[i], marker='o')

    # place grid in back
    # ax.grid(True, linestyle='-', which='major', axis='y',
    #         color='lightgrey', alpha=0.5)
    # ax.set_axisbelow(True)

    # Add titles and labels
    plt.xlabel(xlabel, fontname="sans-serif", fontsize=9)
    plt.ylabel(ylabel, fontname="sans-serif", fontsize=9)
    plt.title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    for label in (ax.get_yticklabels()):
        label.set_fontsize(8)

    # x-ticks x-axis
    ax.set_xticks(ind)
    ax.set_xticklabels(labels, fontname="sans-serif", fontsize=9)
    # ax.set_xticklabels(ind, labels, fontname="sans-serif", fontsize=9)
    for tick in ax.get_xticklabels():
        tick.set_rotation(0)

    return upper_data_bound

def make_boxplot(data, title, xlabel, ylabel, labels, box_colors, box_alpha, figure_size):
    """
    Creates and formats boxplot
    Inputs:
        data - The original data formatted it in a list of numpy arrays
        title/xlabel/ylabel - strings for each label
        labels - list of strings corresponding to each list item in data
        box_colors - list of strings corresponding to the desired color for each
            list item in data
        box_alpha - list of floats from 0 to 1 corresponding to the desired
            tranparency for each list item in data
        figure_size - tuple for figure size
    Outputs: the fig and ax labels for the figure
    """

    fig, ax = plt.subplots(figsize=figure_size, dpi=300)
    medianprops = dict(linewidth=2.5, color='black')
    bp = ax.boxplot(data, notch=0, medianprops=medianprops, labels=labels)  # , patch_artist=True,boxprops=dict(facecolor=color_combine, color=c))
    plt.setp(bp['boxes'], color='black')
    plt.setp(bp['whiskers'], color='black')
    plt.setp(bp['fliers'], color='red', marker='+')
    # Add a horizontal grid to the plot, but make it very light in color
    # so we can use it for reading data values but not be distracting
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

    ax.set_axisbelow(True)  # Hide these grid behind plot objects
    ax.set_title(title, fontname="sans-serif", fontsize=9, fontweight='bold')
    ax.set_xlabel(xlabel, fontname="sans-serif", fontsize=9)
    ax.set_ylabel(ylabel, fontname="sans-serif", fontsize=9)
    # for label in (ax.get_xticklabels() + ax.get_yticklabels()):
    #     label.set_fontsize(8)
    for tick in ax.get_xticklabels():
        tick.set_rotation(0)

    numboxes = len(data)

    for i in range(numboxes):
        box = bp['boxes'][i]
        boxX = []
        boxY = []
        for j in range(5):
            boxX.append(box.get_xdata()[j])
            boxY.append(box.get_ydata()[j])
        box_coords = np.column_stack([boxX, boxY])
        ax.add_patch(Polygon(box_coords, facecolor=box_colors[i],
                     alpha=box_alpha[i], zorder=2))

    for i in range(numboxes):
        x_coordinate = i+1
        y_mean = np.mean(data[i])
        y_std = np.std(data[i])/np.sqrt(data[i].shape[0])
        ax.plot([x_coordinate, x_coordinate], [y_mean-y_std, y_mean+y_std],
                'black', markersize=7, zorder=3)
        ax.plot(x_coordinate, y_mean, 'o', color='w', marker='o',
                markersize=7, markeredgecolor='black', zorder=4)

    return [fig, ax]
