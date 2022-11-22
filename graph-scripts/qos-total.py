#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
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
	pdr_all = []
	delay_all = []
	jitter_all = []

	pdr_mean = 0
	delay_mean = 0
	jitter_mean = 0

	with open(filePath) as data:
		for peers, pdr, plr, delay, jitter, throughput in csv.reader(data,delimiter=','):
			pdr_all.append(int(pdr))
			if math.isnan(float(delay)):
				delay_all.append(0)
				jitter_all.append(0)
			else:
				delay_all.append(float(delay))
				jitter_all.append(float(jitter))
		pdr_mean = np.mean(pdr_all)
		delay_mean = np.mean(delay_all) * 1000
		jitter_mean = np.mean(jitter_all) * 1000

	return QosData(pdr_mean, delay_mean, jitter_mean)

def getData(scenarios):
	data = dict()
	for scenario in scenarios.items():
		runs = glob.glob(scenario[1] + "/*/qos.txt")
		qos = []
		for run in runs:
			qos_run = readFile(run)
			qos.append(qos_run)

		if runs == []:
			print("scenario " + scenario[1] + " has no data")
			qos_result = QosResult(0,0,0,0,0,0)
		else:
			print("scenario {} has {} data points".format(scenario[1], len(runs)))
			qos_mean = np.mean(qos, axis=0)
			qos_std = np.std(qos, axis=0)
			qos_result = QosResult(
				qos_mean[0], qos_std[0], 
				qos_mean[1], qos_std[1],
				qos_mean[2], qos_std[2]
				)

		data[scenario[0]] = qos_result
	print("")
	return data

ScenarioVariant = namedtuple("ScenarioVariant", ["name", "path", "color"])
QosData = namedtuple("QosData", ["pdr", "delay", "jitter"])
QosResult = namedtuple("QosResult", ["pdr", "pdr_std", "delay", "delay_std", "jitter", "jitter_std"])

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

	pdr_mean = [data.pdr for data in data_scenarios.values()]
	pdr_std = [data.pdr_std for data in data_scenarios.values()]
	rect = ax_pdr.bar(ind + variantN*width, pdr_mean, width, yerr=pdr_std, align='center', alpha=1.0, ecolor='black', capsize=6, color=variant.color, linewidth=1.0, edgecolor='black')
	pdr_rectangles.append(rect[0])
	ax_pdr.set(xticks=ind+width*2, xticklabels=list(data_scenarios.keys()))
	ax_pdr.set_xlabel("Number of users", fontsize=17)
	ax_pdr.tick_params(axis="x", labelsize=15)
	ax_pdr.set_yticks(np.arange(0,101,10))
	ax_pdr.set_ylabel("PDR (%)", fontsize=17)
	ax_pdr.tick_params(axis="y", labelsize=15)
	ax_pdr.yaxis.grid(color='gray', linestyle='dotted')

	delay_mean = [data.delay for data in data_scenarios.values()]
	delay_std = [data.delay_std for data in data_scenarios.values()]
	rect = ax_delay.bar(ind + variantN*width, delay_mean, width, yerr=delay_std, align='center', alpha=1.0, ecolor='black', capsize=6, color=variant.color, linewidth=1.0, edgecolor='black')
	delay_rectangles.append(rect[0])
	ax_delay.set(xticks=ind+width*2, xticklabels=list(data_scenarios.keys()))
	ax_delay.set_xlabel("Number of users", fontsize=17)
	ax_delay.tick_params(axis="x", labelsize=15)
	ax_delay.set_yticks(np.arange(0,121,10))
	ax_delay.set_ylabel("Delay (ms)", fontsize=17)
	ax_delay.tick_params(axis="y", labelsize=15)
	ax_delay.yaxis.grid(color='gray', linestyle='dotted')

	jitter_mean = [data.jitter for data in data_scenarios.values()]
	jitter_std = [data.jitter_std for data in data_scenarios.values()]
	rect = ax_jitter.bar(ind + variantN*width, jitter_mean, width, yerr=jitter_std, align='center', alpha=1.0, ecolor='black', capsize=6, color=variant.color, linewidth=1.0, edgecolor='black')
	jitter_rectangles.append(rect[0])
	ax_jitter.set(xticks=ind+width*2, xticklabels=list(data_scenarios.keys()))
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

fig_pdr.savefig("pdr.png", format='png', dpi=300, bbox_inches = "tight")
fig_pdr.savefig("pdr.pdf", format='pdf', dpi=300, bbox_inches = "tight")
fig_delay.savefig("delay.png", format='png', dpi=300, bbox_inches = "tight")
fig_delay.savefig("delay.pdf", format='pdf', dpi=300, bbox_inches = "tight")
fig_jitter.savefig("jitter.png", format='png', dpi=300, bbox_inches = "tight")
fig_jitter.savefig("jitter.pdf", format='pdf', dpi=300, bbox_inches = "tight")
#plt.show()
