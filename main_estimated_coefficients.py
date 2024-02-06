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
    position_file = "./input/df_149_position.csv",
    velocity_file = "./input/df_149_velocity.csv",
    actuator_file = "./input/df_149_actuator.csv")

    basic_params = get_parameters.get_149ship_basic_params()
    mpc_params = get_parameters.set_mpc_params(
        n_horizon = 100, #予測区間の長さ
        t_step = 1, #time step
        n_robust = 1, #制御区間の長さ
        store_full_solution = True, #
        δ_set = 1e-3, #評価関数の舵角の変化量に対する重み
        n_p_set = 1e-3, #評価関数のプロペラ回転数の変化量に対する重み
        δ_max = 45.0*np.pi/180.0, #制御入力として指示できる舵角の最大値
        δ_min = -45.0*np.pi/180.0, #制御入力として指示できる舵角の最小値
        n_p_max = 50.0, #制御入力として指示できるプロペラ回転数の最大値
        n_p_min = 0.0, #制御入力として指示できるプロペラ回転数の最小値
        u_max = 15.0, #状態量の船舶の前後方向速度の最大値
        u_min = 0.0, #状態量の船舶の前後方向速度の最小値
        v_max = 10.0, #状態量の船舶の横方向速度の最大値
        v_min = -10.0, #状態量の船舶の横方向速度の最小値
        control_duration = 100, #制御を行う時間
        model_type = 'continuous' # either 'discrete' or 'continuous'
        )

    dt_now=get_time()
    dirname='./output/output'+dt_now

    df_coef=pd.read_csv(input_files.coefficients_file)
    num_of_samples=df_coef.shape[0]

    average=False
    if average:
        df_coef=df_coef.mean()
        mmg_params = get_parameters.set_mmg_params(0.2931, -0.2753, -0.1385,
            df_coef.R_0, df_coef.X_vv, df_coef.X_vr, df_coef.X_rr, df_coef.X_vvvv,
            df_coef.Y_v, df_coef.Y_r, df_coef.Y_vvv, df_coef.Y_vvr, df_coef.Y_vrr, df_coef.Y_rrr,
            df_coef.N_v, df_coef.N_r, df_coef.N_vvv, df_coef.N_vvr, df_coef.N_vrr, df_coef.N_rrr, basic_params)
        main(input_files, basic_params, mmg_params, mpc_params, dirname)

    else:
        num_of_samples_to_run=30 #何個の微係数推定値セットでMPCを実行するか
        num_list=[ i for i in range(num_of_samples)]
        random.shuffle(num_list)

        for sample_num in num_list[:num_of_samples_to_run]:
            print(sample_num)
            df=df_coef.iloc[sample_num]
            mmg_params = get_parameters.set_mmg_params(0.2931, -0.2753, -0.1385,
                df.R_0, df.X_vv, df.X_vr, df.X_rr, df.X_vvvv,
                df.Y_v, df.Y_r, df.Y_vvv, df.Y_vvr, df.Y_vrr, df.Y_rrr,
                df.N_v, df.N_r, df.N_vvv, df.N_vvr, df.N_vrr, df.N_rrr, basic_params)
            main(input_files, basic_params, mmg_params, mpc_params, dirname, sample_num)

    print("Program terminated successfully.")