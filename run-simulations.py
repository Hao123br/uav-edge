#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sem
import argparse

ns_path = './'
script = 'simulation'
campaign_dir = ns_path + 'sem'
results_dir = ns_path + 'results'

parser = argparse.ArgumentParser(description='SEM script')
parser.add_argument('-o', '--overwrite', action='store_true',
                    help='Overwrite previous campaign')
args = parser.parse_args()

campaign = sem.CampaignManager.new(ns_path, script, campaign_dir,
            overwrite=args.overwrite, check_repo=True)
print(campaign)

param_combinations = {
    'seed' : 10000,
    'numSCs' : 1,
    'numUAVs' : 0,
    'numUEs' : [100, 200, 300],
    'remMode' : 0
}

campaign.run_missing_simulations(sem.list_param_combinations(param_combinations),1)

result_param = {
    'numSCs' : [1],
    'numUEs' : [100, 200, 300]
}

campaign.save_to_folders(result_param, results_dir, 1)

param_combinations = {
    'seed' : 10000,
    'numSCs' : 0,
    'numUAVs' : 8,
    'numUEs' : [100, 200, 300],
    'remMode' : 0
}

campaign.run_missing_simulations(sem.list_param_combinations(param_combinations),1)

result_param = {
    'numUAVs' : [8],
    'numUEs' : [100, 200, 300]
}

campaign.save_to_folders(result_param, results_dir, 1)
