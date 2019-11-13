# 3rd-party packages
import matplotlib.pyplot as plt


#
# @brief      Saves a plot given by a figure handle to given base file name
#
# @param      fig              The figure handle to save
# @param      shouldSavePlots  Flag to turn the plot saving on/off
# @param      baseSaveFName    The base directory file name for output plot
# @param      plotTitle        specific plot title string to add to
#                              baseSaveFName
# @param      useTightLayout   Controls the use of tight plot layouts -
#                              Defaults to True.
#
def savePlot(fig, shouldSavePlots, baseSaveFName, plotTitle,
             useTightLayout=True):

    if shouldSavePlots:

        saveFName = baseSaveFName + '-' + plotTitle + '.png'
        if useTightLayout:
            plt.tight_layout()
        plt.savefig(saveFName, dpi=500)

        print('wrote figure to: ', saveFName)

        # close the figure as it is no longer needed and can cause the
        # following error:
        #
        # RuntimeWarning: More than 20 figures have been
        # opened. Figures created through the pyplot interface
        # (`matplotlib.pyplot.figure`) are retained until explicitly closed and
        # may consume too much memory. (To control this warning, see the
        # rcParam `figure.max_open_warning`).
        plt.close(fig)
