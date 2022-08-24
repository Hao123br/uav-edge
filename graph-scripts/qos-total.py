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
			qos_mean = QosData(0,0,0)
		else:
			print("scenario {} has {} data points".format(scenario[1], len(runs)))
			qos_mean = QosData._make(np.mean(qos, axis=0))

		data[scenario[0]] = qos_mean
	print("")
	return data

ScenarioVariant = namedtuple("ScenarioVariant", ["name", "path"])
QosData = namedtuple("QosData", ["pdr", "delay", "jitter"])

scenario_variants = [ScenarioVariant("No UAVs", "../results/numSCs=1"),
					 ScenarioVariant("With UAVs", "../results/numUAVs=8")]

fig = plt.figure(figsize=(13,3))
ax_pdr = fig.add_subplot(131)
ax_delay = fig.add_subplot(132)
ax_jitter = fig.add_subplot(133)

for variant in scenario_variants:
	scenarios = getScenarios(variant.path)
	data_scenarios = getData(scenarios)

	pdr_data = [data.pdr for data in data_scenarios.values()]
	ax_pdr.plot(data_scenarios.keys(), pdr_data, label=variant.name)
	ax_pdr.set_title("PDR")
	ax_pdr.set(xticks=list(data_scenarios.keys()), xlabel="Number of users")
	ax_pdr.set(ylim=[0,100], yticks=np.arange(0,101,10), ylabel="%")
	ax_pdr.legend()

	delay_data = [data.delay for data in data_scenarios.values()]
	ax_delay.plot(data_scenarios.keys(), delay_data, label=variant.name)
	ax_delay.set_title("Delay")
	ax_delay.set(xticks=list(data_scenarios.keys()), xlabel="Number of users")
	ax_delay.set(ylim=[0,120], yticks=np.arange(0,121,20), ylabel="milliseconds")
	ax_delay.legend()

	jitter_data = [data.jitter for data in data_scenarios.values()]
	ax_jitter.plot(data_scenarios.keys(), jitter_data, label=variant.name)
	ax_jitter.set_title("Jitter")
	ax_jitter.set(xticks=list(data_scenarios.keys()), xlabel="Number of users")
	ax_jitter.set(ylim=[0,20], yticks=np.arange(0,26,5), ylabel="milliseconds")
	ax_jitter.legend()

plt.savefig("qos.png", format='png', dpi=500, bbox_inches = "tight")
#plt.show()
