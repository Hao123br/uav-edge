#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import csv
import glob
from collections import namedtuple

def getScenarios(path):
	scenarios = dict()

	directories = glob.glob(path + "/numUEs=*")
	directories.sort()
	for directory in directories:
		subdir = directory.split('/')[-1]
		numUEs = int(subdir.split('=')[-1])
		scenarios[numUEs] = directory

	return scenarios


def readFile(filePath):
	result = pd.read_csv(filePath, sep=',', names=['peers', 'pdr', 'plr', 'delay',
	'jitter', 'throughput'], usecols=['pdr', 'delay', 'jitter'])
	result = result.mean()
	result['delay'] = result['delay'] * 1000
	result['jitter'] = result['jitter'] * 1000

	return result

def getData(scenarios):
	data = []
	for scenario in scenarios.items():
		runs = glob.glob(scenario[1] + "/*/qos.txt")
		qos = []
		for run in runs:
			qos_run = readFile(run)
			qos.append(qos_run)

		if runs == []:
			print("scenario " + scenario[1] + " has no data")
			zeroes = pd.Series(data=[0, 0, 0], index=['pdr', 'delay','jitter'])
			d = {'mean' : zeroes, 'std' : zeroes}
		else:
			print("scenario {} has {} data points".format(scenario[1], len(runs)))
			qos = pd.DataFrame(qos)
			d = {'mean' : qos.mean(), 'std' : qos.std()}
		qos_result = pd.DataFrame(d)

		data.append(qos_result)
	data = pd.concat(data)
	print("")
	return data

ScenarioVariant = namedtuple("ScenarioVariant", ["name", "path", "color"])

scenario_variants = [ScenarioVariant("No UAVs", "../results/numSCs=1", "#ffc0cb"),
					 ScenarioVariant("With UAVs", "../results/numUAVs=8", "#c1e7ff")]

fig_pdr = plt.figure()
ax_pdr = fig_pdr.subplots()

fig_delay = plt.figure()
ax_delay = fig_delay.subplots()

fig_jitter = plt.figure()
ax_jitter = fig_jitter.subplots()

ind = np.arange(3)
width = 0.15       # the width of the bars
variantN = 0
pdr_rectangles = []
delay_rectangles = []
jitter_rectangles = []

for variant in scenario_variants:
	scenarios = getScenarios(variant.path)
	data_scenarios = getData(scenarios)

	pdr = data_scenarios.loc['pdr']
	pdr_mean = pdr['mean']
	pdr_std = pdr['std']
	rect = ax_pdr.bar(ind + variantN*width, pdr_mean, width, yerr=pdr_std, align='center', alpha=1.0, ecolor='black', capsize=6, color=variant.color, linewidth=1.0, edgecolor='black')
	pdr_rectangles.append(rect[0])
	ax_pdr.set(xticks=ind+width*2, xticklabels=list(scenarios.keys()))
	ax_pdr.set_xlabel("Number of users", fontsize=17)
	ax_pdr.tick_params(axis="x", labelsize=15)
	ax_pdr.set_yticks(np.arange(0,101,10))
	ax_pdr.set_ylabel("PDR (%)", fontsize=17)
	ax_pdr.tick_params(axis="y", labelsize=15)
	ax_pdr.yaxis.grid(color='gray', linestyle='dotted')

	delay = data_scenarios.loc['delay']
	delay_mean = delay['mean']
	delay_std = delay['std']
	rect = ax_delay.bar(ind + variantN*width, delay_mean, width, yerr=delay_std, align='center', alpha=1.0, ecolor='black', capsize=6, color=variant.color, linewidth=1.0, edgecolor='black')
	delay_rectangles.append(rect[0])
	ax_delay.set(xticks=ind+width*2, xticklabels=list(scenarios.keys()))
	ax_delay.set_xlabel("Number of users", fontsize=17)
	ax_delay.tick_params(axis="x", labelsize=15)
	ax_delay.set_yticks(np.arange(0,121,10))
	ax_delay.set_ylabel("Delay (ms)", fontsize=17)
	ax_delay.tick_params(axis="y", labelsize=15)
	ax_delay.yaxis.grid(color='gray', linestyle='dotted')

	jitter = data_scenarios.loc['jitter']
	jitter_mean = jitter['mean']
	jitter_std = jitter['std']
	rect = ax_jitter.bar(ind + variantN*width, jitter_mean, width, yerr=jitter_std, align='center', alpha=1.0, ecolor='black', capsize=6, color=variant.color, linewidth=1.0, edgecolor='black')
	jitter_rectangles.append(rect[0])
	ax_jitter.set(xticks=ind+width*2, xticklabels=list(scenarios.keys()))
	ax_jitter.set_xlabel("Number of users", fontsize=17)
	ax_jitter.tick_params(axis="x", labelsize=15)
	ax_jitter.set_yticks(np.arange(0,26,5))
	ax_jitter.set_ylabel("Jitter (ms)", fontsize=17)
	ax_jitter.tick_params(axis="y", labelsize=15)
	ax_jitter.yaxis.grid(color='gray', linestyle='dotted')

	variantN += 1

labels = [variant.name for variant in scenario_variants]
ax_pdr.legend(pdr_rectangles, labels, loc='upper center', bbox_to_anchor=(0., 1.12, 0.95, .102),
	    ncol=3, borderaxespad=0.,  prop={'size': 14})
ax_delay.legend(delay_rectangles, labels, loc='upper center', bbox_to_anchor=(0., 1.12, 0.95, .102),
	    ncol=3, borderaxespad=0.,  prop={'size': 14})
ax_jitter.legend(jitter_rectangles, labels, loc='upper center', bbox_to_anchor=(0., 1.12, 0.95, .102),
	    ncol=3, borderaxespad=0.,  prop={'size': 14})

fig_pdr.savefig("pdr-vs-users.png", format='png', dpi=300, bbox_inches = "tight")
fig_pdr.savefig("pdr-vs-users.pdf", format='pdf', dpi=300, bbox_inches = "tight")
fig_delay.savefig("delay-vs-users.png", format='png', dpi=300, bbox_inches = "tight")
fig_delay.savefig("delay-vs-users.pdf", format='pdf', dpi=300, bbox_inches = "tight")
fig_jitter.savefig("jitter-vs-users.png", format='png', dpi=300, bbox_inches = "tight")
fig_jitter.savefig("jitter-vs-users.pdf", format='pdf', dpi=300, bbox_inches = "tight")
#plt.show()
