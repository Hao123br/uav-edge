#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sem
import argparse

ns_path = './'
script = 'ns3.36.1-simulation-optimized'
campaign_dir = ns_path + 'sem'
results_dir = ns_path + 'results'
nRuns = 30

parser = argparse.ArgumentParser(description='SEM script')
parser.add_argument('-o', '--overwrite', action='store_true',
                    help='Overwrite previous campaign')
args = parser.parse_args()

campaign = sem.CampaignManager.new(ns_path, script, campaign_dir,
            overwrite=args.overwrite, check_repo=False)

print("running simulations with eNBs")
param_combinations = {
#	'RngSeed' : 4200,
    'numENBs' : 1,
    'numUAVs' : 8,
	'algo' : ['no-uavs', 'topsis'],
    'numUEs' : [100, 200, 300],
}

campaign.run_missing_simulations(sem.list_param_combinations(param_combinations),
		runs=nRuns, stop_on_errors=False)

result_param = { 
    'numENBs' : [1],
    'numUEs' : [100, 200, 300]
}

print("exporting results")
campaign.save_to_folders(result_param, results_dir, nRuns)

print("running simulations without eNBs")
param_combinations['numENBs'] = [0]
param_combinations['algo'] = ['topsis']

campaign.run_missing_simulations(sem.list_param_combinations(param_combinations),
		runs=nRuns, stop_on_errors=False)

result_param['numENBs'] = [0]

print("exporting results")
campaign.save_to_folders(result_param, results_dir, nRuns)
