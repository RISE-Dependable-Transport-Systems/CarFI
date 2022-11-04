import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator


class Analyzer:
    def __init__(self, output_path):
        self.output_path = output_path

    def safety_violation_time(self, config):
        df = pd.read_csv(self.output_path)
        result = df.groupby(['sim_round'])['sim_time'].max()
        ax = plt.figure().gca()
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))

        ax = sns.histplot(result, bins=10)
        ax.set(ylabel='quantity', xlabel='safety violation time')
        plt.savefig('./out/safety_violation_time.png')
        plt.clf()

    def corr(self, params):
        df = pd.read_csv(self.output_path).groupby(['sim_round'])[params].max()
        result = df.corr()
        sns.heatmap(result, cmap="YlGnBu", annot=True)
        plt.savefig('./out/corr.png')
        plt.clf()

    def successful_missions(self, config):
        df = pd.read_csv(
            self.output_path).groupby(['sim_round'])['safety_violated'].any().to_frame().reset_index()

        sim_rounds = len(df)
        result = np.zeros((sim_rounds, 2))

        for i in range(1, sim_rounds+1):
            df_p = df[df['sim_round'] <= i]
            result[i-1] = [i, df_p['safety_violated'].sum()]
        result = pd.DataFrame(data=result, columns=[
            'simulation round', 'successfull missions'])
        # plt.xticks(range(1, sim_rounds+1, ))
        ax = plt.figure().gca()
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        sns.regplot(data=result, x='simulation round',
                    y='successfull missions')
        plt.savefig('./out/mission_success_rate.png')
        plt.clf()
