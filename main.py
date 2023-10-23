from do_mpc.data import load_results
import get_parameters
from MMG_maneuvering_model import MMG_model
from MPC_execute import MPC_exe
from make_output_files import make_outputs
from datetime import datetime
import pandas as pd
import random
import numpy as np

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
    coefficients_file = "./input/149_coefficient_sample.csv", #参照する微係数のファイル名
    position_file = "./input/df_position.csv", #目標軌道の座標・方位角が入っているファイル名
    velocity_file = "./input/df_velocities.csv", #目標軌道を作成した時の船舶の速度・角速度が入ったファイル名
    actuator_file = "./input/df_actuator.csv") #目標軌道を作成した時の舵角・プロペラ回転数が入ったファイル名

    basic_params = get_parameters.set_basic_params(
        ρ=1.025, #海水密度
        L_pp = 3.50,  # 船長Lpp[m]
        B = 0.57,  # 船幅[m]
        d = 0.16,  # 喫水[m]
        nabla = 0.146,  # 排水量[m^3]
        x_G = 0.0,  # 重心位置[m]
        C_b = 0.738,  # 方形係数[-]
        D_p = 0.120,  # プロペラ直径[m]
        H_R = 0.13,  # 舵高さ[m]
        A_R = 0.01867,  # 舵断面積[m^2]
        t_P = 0.175,  # 推力減少率
        w_P0 = 0.168,  # 有効伴流率
        m_x_dash = 0.049,  # 付加質量x(無次元)
        m_y_dash = 0.1451,  # 付加質量y(無次元)
        J_z_dash = 0.0086,  # 付加質量Izz(無次元)
        t_R = 0.25,  # 操縦抵抗減少率
        x_R_dash = -0.5,  # 舵の相対位置
        a_H = 0.5,  # 舵力増加係数
        x_H_dash = -0.45,  # 舵力増分作用位置
        γ_R_minus = 0.45,  # 整流係数
        γ_R_plus = 0.45,  # 整流係数
        l_r_dash = -0.9,  # 船長に対する舵位置
        x_P_dash = -0.690,  # 船長に対するプロペラ位置
        ϵ = 1.1,  # プロペラ・舵位置伴流係数比
        κ = 0.545, # 修正係数
        f_α = 2.656  # 直圧力勾配係数
        )

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

    average=False
    if average:
        df=df_coef.mean()
        mmg_params = get_parameters.set_mmg_params(
            k_0=0.2931, 
            k_1=-0.2753, 
            k_2=-0.1385,
            R_0_dash=df.R_0, 
            X_vv_dash=df.X_vv, 
            X_vr_dash=df.X_vr, 
            X_rr_dash=df.X_rr, 
            X_vvvv_dash=df.X_vvvv,
            Y_v_dash=df.Y_v, 
            Y_r_dash=df.Y_r, 
            Y_vvv_dash=df.Y_vvv, 
            Y_vvr_dash=df.Y_vvr, 
            Y_vrr_dash=df.Y_vrr, 
            Y_rrr_dash=df.Y_rrr,
            N_v_dash=df.N_v, 
            N_r_dash=df.N_r, 
            N_vvv_dash=df.N_vvv, 
            N_vvr_dash=df.N_vvr, 
            N_vrr_dash=df.N_vrr, 
            N_rrr_dash=df.N_rrr, 
            basic_params=basic_params)
        main(input_files, basic_params, mmg_params, mpc_params, dirname)

    else:
        num_list=[ i for i in range(1000)]
        random.shuffle(num_list)

        for sample_num in num_list[:5]:
            print(sample_num)
            df=df_coef.iloc[sample_num]
            mmg_params = get_parameters.set_mmg_params(
                k_0=0.2931, 
                k_1=-0.2753, 
                k_2=-0.1385,
                R_0_dash=df.R_0, 
                X_vv_dash=df.X_vv, 
                X_vr_dash=df.X_vr, 
                X_rr_dash=df.X_rr, 
                X_vvvv_dash=df.X_vvvv,
                Y_v_dash=df.Y_v, 
                Y_r_dash=df.Y_r, 
                Y_vvv_dash=df.Y_vvv, 
                Y_vvr_dash=df.Y_vvr, 
                Y_vrr_dash=df.Y_vrr, 
                Y_rrr_dash=df.Y_rrr,
                N_v_dash=df.N_v, 
                N_r_dash=df.N_r, 
                N_vvv_dash=df.N_vvv, 
                N_vvr_dash=df.N_vvr, 
                N_vrr_dash=df.N_vrr, 
                N_rrr_dash=df.N_rrr, 
                basic_params=basic_params)
            main(input_files, basic_params, mmg_params, mpc_params, dirname, sample_num)
    
    print("Program terminated successfully.")