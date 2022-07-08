#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from matplotlib import pyplot as plt
import csv
import argparse

class QosData:

	def __init__(self, pdr, delay, jitter):
		self.pdr = pdr
		self.delay = delay
		self.jitter = jitter

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

	print("pdr: {:.1f} delay: {:.1f} jitter: {:.1f}".format(pdr_mean, delay_mean,
		jitter_mean))
	return QosData(pdr_mean, delay_mean, jitter_mean)

parser = argparse.ArgumentParser(description='qos means script')
parser.add_argument('path')
args = parser.parse_args()
noUAVs = readFile(args.path)
