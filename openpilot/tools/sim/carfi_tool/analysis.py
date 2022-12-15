from matplotlib.ticker import MaxNLocator
import pandas as pd
import matplotlib
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np


matplotlib.use('Agg')


def safety_violation_time(output_path):
    plt.figure(figsize=(6, 6))
    df = pd.read_csv(output_path)
    df_op_startup = df[df['op started'] == True].groupby(['experiment'])['simulation time'].min()
    df_violation_time = df[df['safety violated'] == True].groupby(['experiment'])['simulation time'].min()
    result = df_violation_time.subtract(df_op_startup)
    ax = plt.figure().gca()
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    ax = sns.histplot(result, bins=10)
    ax.set(ylabel='quantity', xlabel='safety violation time')
    plt.savefig('./out/safety_violation_delay.png')
    plt.clf()


def corr(output_path):
    plt.figure(figsize=(14, 14))
    df = pd.read_csv(output_path).groupby(['experiment']).max()
    result = df.corr(numeric_only=True)
    sns.heatmap(result, cmap="YlGnBu", annot=False)
    plt.savefig('./out/corr.png')
    plt.clf()


def successful_missions(output_path):
    plt.figure(figsize=(6, 6))
    df = pd.read_csv(
        output_path).groupby(['experiment'])['crashed'].any().to_frame().reset_index()

    sim_rounds = len(df)
    result = np.zeros((sim_rounds, 2))

    for i in range(1, sim_rounds+1):
        df_p = df[df['experiment'] <= i]
        result[i-1] = [i, i-df_p['crashed'].sum()]
    result = pd.DataFrame(data=result, columns=[
        'experiment', 'successfull missions'])
    # plt.xticks(range(1, sim_rounds+1, ))
    ax = plt.figure().gca()
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    sns.regplot(data=result, x='experiment',
                y='successfull missions')
    plt.savefig('./out/mission_success_rate.png')
    plt.clf()
