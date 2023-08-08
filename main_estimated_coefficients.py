from do_mpc.data import load_results
import get_parameters
from MMG_maneuvering_model import MMG_model
from MPC_execute import MPC_exe
from make_output_files import make_outputs
from datetime import datetime
import pandas as pd
import random

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

def main(input_files, basic_params, mmg_params, mpc_params, dirname, sample_num=None):

    MPC=MPC_exe(basic_params, mmg_params, mpc_params, input_files)
    MPC.main()

    results = load_results('./results/results.pkl')

    Output_Files = make_outputs(results, mmg_params, mpc_params, input_files, dirname, sample_num)
    Output_Files.main()

if __name__ == '__main__':

    input_files = get_parameters.set_input_file_names(
    coefficients_file = "./input/149_coefficient_sample.csv",
    position_file = "./input/df_position.csv",
    velocity_file = "./input/df_velocities.csv",
    actuator_file = "./input/df_actuator.csv")

    basic_params = get_parameters.get_149ship_basic_params()
    mpc_params = get_parameters.get_default_mpc_params()

    dt_now=get_time()
    dirname='./output/output'+dt_now

    df_coef=pd.read_csv(input_files.coefficients_file)

    average=False
    if average:
        df_coef=df_coef.mean()
        mmg_params = get_parameters.set_mmg_params(0.2931, -0.2753, -0.1385,
            df_coef.R_0, df_coef.X_vv, df_coef.X_vr, df_coef.X_rr, df_coef.X_vvvv,
            df_coef.Y_v, df_coef.Y_r, df_coef.Y_vvv, df_coef.Y_vvr, df_coef.Y_vrr, df_coef.Y_rrr,
            df_coef.N_v, df_coef.N_r, df_coef.N_vvv, df_coef.N_vvr, df_coef.N_vrr, df_coef.N_rrr, basic_params)
        main(input_files, basic_params, mmg_params, mpc_params, dirname)

    else:
        num_list=[ i for i in range(1000)]
        random.shuffle(num_list)

        for sample_num in num_list[:5]:
            print(sample_num)
            df=df_coef.iloc[sample_num]
            mmg_params = get_parameters.set_mmg_params(0.2931, -0.2753, -0.1385,
                df.R_0, df.X_vv, df.X_vr, df.X_rr, df.X_vvvv,
                df.Y_v, df.Y_r, df.Y_vvv, df.Y_vvr, df.Y_vrr, df.Y_rrr,
                df.N_v, df.N_r, df.N_vvv, df.N_vvr, df.N_vrr, df.N_rrr, basic_params)
            main(input_files, basic_params, mmg_params, mpc_params, dirname, sample_num)
    
    print("Program terminated successfully.")