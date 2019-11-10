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
#
def savePlot(fig, shouldSavePlots, baseSaveFName, plotTitle):

    if shouldSavePlots:

        saveFName = baseSaveFName + '-' + plotTitle + '.png'
        fig = plt.gcf()
        fig.canvas.manager.full_screen_toggle()
        fig.show()
        fig.set_size_inches((11, 8.5), forward=False)
        plt.savefig(saveFName, dpi=500)

        print('wrote figure to: ', saveFName)
