#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sem
import argparse

ns_path = './'
script = 'simulation'
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
    'seed' : 4200,
    'numENBs' : 2,
    'numUAVs' : 16,
    'numUEs' : [100],
}

campaign.run_missing_simulations(sem.list_param_combinations(param_combinations),
        runs=nRuns, stop_on_errors=False)

result_param = {
    'numENBs': [2],
    'numUAVs': [16],
    'numUEs' : [100]
}

print("exporting results")
campaign.save_to_folders(result_param, results_dir, nRuns)
