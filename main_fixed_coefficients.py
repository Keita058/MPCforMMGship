from do_mpc.data import load_results
import get_parameters
from MMG_maneuvering_model import MMG_model
from Model_predictive_control import ModelPredictiveControl
from make_output_files import make_outputs
from make_graph import Make_Graphs
from datetime import datetime
import pandas as pd
import random
import numpy as np
import os

def get_time():
    dt_now=datetime.now()
    year=str(dt_now.year)
    month=str(dt_now.month)
    day=str(dt_now.day)
    hour=str(dt_now.hour)
    minute=str(dt_now.minute)
    second=str(dt_now.second)
    now_time=''
    for strings in [year,month,day,hour,minute,second]:
        if len(strings)==1:
            strings='0'+strings
        now_time=now_time+strings
    return now_time

def output_dirs(dir_path):
    files=os.listdir(dir_path)
    res_dirs_list=list()
    for file in files:
        file_path=dir_path+'/'+file
        if os.path.isdir(file_path):
            res_dirs_list.append(file_path)
    if not res_dirs_list:
        res_dirs_list.append(dir_path)
    return res_dirs_list


def main(input_files, basic_params, mmg_params, mpc_params, dir_path, sample_num=None):

    MPC=ModelPredictiveControl(basic_params, mmg_params, mpc_params, input_files)
    MMG=MMG_model(basic_params, mmg_params)
    MPC.main(MMG)

    results = load_results('./results/results.pkl')

    Output_Files = make_outputs(results, basic_params, mmg_params, mpc_params, input_files, dir_path, sample_num)
    Output_Files.main()

    output_dirs_list=output_dirs(dir_path)
    for output_dir_path in output_dirs_list:
        Graphs=Make_Graphs(output_dir_path)
        Graphs.main()


if __name__ == '__main__':

    input_files = get_parameters.set_input_file_names(
    coefficients_file = "./input/149_coefficient_sample.csv",
    position_file = "./input/df_position.csv",
    velocity_file = "./input/df_velocities.csv",
    actuator_file = "./input/df_actuator.csv")

    basic_params = get_parameters.get_KVLCC2_L7model_basic_params()

    mmg_params = get_parameters.get_KVLCC2_L7model_mmg_params()

    mpc_params = get_parameters.get_default_mpc_params()

    dt_now=get_time()
    dirname='./output/output'+dt_now

    main(input_files, basic_params, mmg_params, mpc_params, dirname)

    print("Program terminated successfully.")