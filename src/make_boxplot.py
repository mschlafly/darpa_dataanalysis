
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def add_stats(data,sig_matrix,ax):

    # takes
        # data (formatted for boxplot)-- this is used to find the maximum value so that the significance lines are placed above
        # sig_matrix -- numpy array with row=each significant pair
                                        #col: factor1(0-N) factor2(0-N) p-value
    # print(sig_matrix.shape[0])
    if sig_matrix.shape[0]==0:
        # print('exited')
        return

    [ymin,ymax]=ax.get_ylim()
    min_dist = (ymax-ymin)/30.0
    sig_fill = np.zeros((10,10))
    for i in range(sig_matrix.shape[0]):
        fac1 = int(sig_matrix[i,0])
        fac2 = int(sig_matrix[i,1])
        pval = sig_matrix[i,2]
        fac1_max = np.max(data[:,fac1])
        fac2_max = np.max(data[:,fac2])
        if fac1_max>fac2_max:
            y_topline = fac1_max+min_dist
        else:
            y_topline = fac2_max+min_dist

        if fac1>fac2:
            step = -1
        else:
            step = 1
        found = False
        while found==False:
            # Check if there is a line at height y_topline
            found=True
            y_topline += min_dist
            for ii in range(fac1,fac2+1,step):
                for j in range(i):
                    if abs(y_topline-sig_fill[j,ii])<min_dist:
                        found=False
                        break
        # fill array for later
        for ii in range(fac1,fac2+1,step):
            sig_fill[i,ii] = y_topline

        ax.plot([fac1+1,fac1+1,fac2+1,fac2+1],
                [fac1_max+min_dist,y_topline,y_topline,fac2_max+min_dist],
                 '-k', markersize=5)
        xloc = fac1+1.0+((fac2-fac1)/2.0)
        found = False
        if abs(round(xloc)-xloc)<.3:
            ii = int(round(xloc)-1)
            for j in range(i):
                if sig_fill[j,ii]>y_topline:
                    found = True
        if found==True:
            xloc -= (min_dist/4)
        yloc = y_topline - (min_dist/3)
        if pval<0.001:
            ax.text(xloc,yloc,'***',horizontalalignment='center',fontsize=8,fontweight='bold')
        elif pval<0.01:
            ax.text(xloc,yloc,'**',horizontalalignment='center',fontsize=8,fontweight='bold')
        elif pval<0.05:
            ax.text(xloc,yloc,'*',horizontalalignment='center',fontsize=8,fontweight='bold')

        if i==3:
            return
    return

def make_boxplot(data,title,xlabel,ylabel,labels,box_colors,box_alpha,figure_size):

    fig, ax = plt.subplots(figsize=figure_size,dpi=150)
    medianprops = dict(linewidth=2.5, color='black')
    bp = ax.boxplot(data, notch=0, medianprops=medianprops, labels=labels)#, patch_artist=True,boxprops=dict(facecolor=color_combine, color=c))
    plt.setp(bp['boxes'], color='black')
    plt.setp(bp['whiskers'], color='black')
    plt.setp(bp['fliers'], color='red', marker='+')
    # Add a horizontal grid to the plot, but make it very light in color
    # so we can use it for reading data values but not be distracting
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                   alpha=0.5)

    ax.set_axisbelow(True) # Hide these grid behind plot objects
    ax.set_title(title)#, fontsize=10, fontweight='bold')
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    # for label in (ax.get_xticklabels() + ax.get_yticklabels()):
    #     label.set_fontsize(8)
    for tick in ax.get_xticklabels():
        tick.set_rotation(45)
    fig.subplots_adjust(bottom=0.2)

    numboxes = data.shape[1]

    medians = np.empty(numboxes)
    for i in range(numboxes):
        box = bp['boxes'][i]
        boxX = []
        boxY = []
        for j in range(5):
            boxX.append(box.get_xdata()[j])
            boxY.append(box.get_ydata()[j])
        box_coords = np.column_stack([boxX, boxY])
        ax.add_patch(Polygon(box_coords, facecolor=box_colors[i], alpha=box_alpha[i]))

    x_coordinates = np.arange(1,numboxes+1)# array([1,2,3,4,5,6,7,8,9,10])
    y_coordinates = np.mean(data,axis=0)
    std_all = np.std(data,axis=0)/np.sqrt(data.shape[0])
    for i in range(numboxes):
        ax.plot([x_coordinates[i],x_coordinates[i]],
                [y_coordinates[i]-std_all[i],y_coordinates[i]+std_all[i]]
                ,'black',markersize=7,zorder=2)
    ax.plot(x_coordinates, y_coordinates, 'o',
                 color='w', marker='o', markersize=7, markeredgecolor='black',zorder=3)#, linewidth=0)


    return [fig,ax]
