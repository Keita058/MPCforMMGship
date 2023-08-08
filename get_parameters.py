import dataclasses
import numpy as np

@dataclasses.dataclass
class InputFileNames:
    coefficients_file: str
    position_file: str
    velocity_file: str
    actuator_file: str

def set_input_file_names(coefficients_file, position_file, velocity_file, actuator_file):
    return InputFileNames(coefficients_file=coefficients_file,
                        position_file=position_file,
                        velocity_file=velocity_file,
                        actuator_file=actuator_file)

@dataclasses.dataclass
class BasicParams:
    ρ: float  # 海水密度
    L_pp: float  # 船長Lpp[m]
    B:float  # 船幅[m]
    d:float  # 喫水[m]
    nabla:float  # 排水量[m^3]
    x_G:float  # 重心位置[m]
    C_b:float  # 方形係数[-]
    D_p:float  # プロペラ直径[m]
    H_R:float  # 舵高さ[m]
    A_R:float  # 舵断面積[m^2]
    t_P:float  # 推力減少率
    w_P0:float  # 有効伴流率
    m_x_dash:float  # 付加質量x(無次元)
    m_y_dash: float  # 付加質量y(無次元)
    J_z_dash: float  # 付加質量Izz(無次元)
    t_R: float  # 操縦抵抗減少率
    x_R_dash: float  # 舵の相対位置
    a_H: float  # 舵力増加係数
    x_H_dash:float  # 舵力増分作用位置
    γ_R_minus: float  # 整流係数
    γ_R_plus: float  # 整流係数
    l_r_dash: float  # 船長に対する舵位置
    x_P_dash: float  # 船長に対するプロペラ位置
    ϵ: float  # プロペラ・舵位置伴流係数比
    κ: float  # 修正係数
    f_α: float  # 直圧力勾配係数
    m: float
    I_zG: float
    η: float
    m_x: float
    m_y: float
    J_z: float
    x_H: float
    x_R: float


def set_basic_params(ρ, L_pp, B, d, nabla, x_G, C_b, D_p, H_R, A_R, t_P, w_P0, 
    m_x_dash, m_y_dash, J_z_dash, t_R, x_R_dash, a_H, x_H_dash,
    γ_R_minus, γ_R_plus, l_r_dash, x_P_dash, ϵ, κ, f_α):
    return BasicParams(ρ=ρ,
                        L_pp=L_pp, 
                        B=B, 
                        d=d, 
                        nabla=nabla, 
                        x_G=x_G,
                        C_b=C_b, 
                        D_p=D_p, 
                        H_R=H_R, 
                        A_R=A_R, 
                        t_P=t_P, 
                        w_P0=w_P0, 
                        m_x_dash=m_x_dash, 
                        m_y_dash=m_y_dash, 
                        J_z_dash=J_z_dash, 
                        t_R=t_R, 
                        x_R_dash=x_R_dash, 
                        a_H=a_H, 
                        x_H_dash=x_H_dash, 
                        γ_R_minus=γ_R_minus, 
                        γ_R_plus=γ_R_plus, 
                        l_r_dash=l_r_dash, 
                        x_P_dash=x_P_dash, 
                        ϵ=ϵ, 
                        κ=κ, 
                        f_α=f_α,
                        m=ρ * nabla,
                        I_zG=ρ * nabla * ((0.25 * L_pp) ** 2),
                        η=D_p / H_R,
                        m_x=(0.5 * ρ * (L_pp ** 2) * d) * m_x_dash,
                        m_y=(0.5 * ρ * (L_pp ** 2) * d) * m_y_dash,
                        J_z=(0.5 * ρ * (L_pp ** 4) * d) * J_z_dash,
                        x_H=x_H_dash * L_pp,
                        x_R=x_R_dash*L_pp
    )

@dataclasses.dataclass
class MPCParams:
    n_horizon: int
    t_step: int
    n_robust: int
    store_full_solution: bool
    δ_set: float
    n_p_set: float
    δ_max: float
    δ_min: float
    n_p_max: float
    n_p_min: float
    u_max: float
    u_min: float
    v_max: float
    v_min: float
    control_duration: int
    model_type: str


def set_mpc_params(n_horizon, t_step, n_robust, store_full_solution, 
    δ_set, n_p_set, δ_max, δ_min, n_p_max, n_p_min, 
    u_max, u_min, v_max, v_min, control_duration,model_type):
    return MPCParams(
        n_horizon=n_horizon,
        t_step=t_step,
        n_robust= n_robust,
        store_full_solution=store_full_solution,
        δ_set=δ_set,
        n_p_set=n_p_set,
        δ_max=δ_max,
        δ_min=δ_min,
        n_p_max=n_p_max,
        n_p_min=n_p_min,
        u_max=u_max,
        u_min=u_min,
        v_max=v_max,
        v_min=v_min,
        control_duration=control_duration,
        model_type=model_type
        )

@dataclasses.dataclass
class MMGManeuveringParams:
    k_0: float
    k_1: float
    k_2: float
    R_0_dash: float
    X_vv_dash: float
    X_vr_dash: float
    X_rr_dash: float
    X_vvvv_dash: float
    Y_v_dash: float
    Y_r_dash: float
    Y_vvv_dash: float
    Y_vvr_dash: float
    Y_vrr_dash: float
    Y_rrr_dash: float
    N_v_dash: float
    N_r_dash: float
    N_vvv_dash: float
    N_vvr_dash: float
    N_vrr_dash: float
    N_rrr_dash: float
    R_0: float

def set_mmg_params(k_0, k_1, k_2, 
    R_0_dash, X_vv_dash, X_vr_dash, X_rr_dash, X_vvvv_dash, 
    Y_v_dash, Y_r_dash, Y_vvv_dash, Y_vvr_dash, Y_vrr_dash, Y_rrr_dash, 
    N_v_dash, N_r_dash, N_vvv_dash, N_vvr_dash, N_vrr_dash, N_rrr_dash,
    basic_params
    ):
    return MMGManeuveringParams(
        k_0=k_0, 
        k_1=k_1, 
        k_2=k_2, 
        R_0_dash=R_0_dash, 
        X_vv_dash=X_vv_dash,
        X_vr_dash=X_vr_dash,
        X_rr_dash=X_rr_dash,
        X_vvvv_dash=X_vvvv_dash,
        Y_v_dash=Y_v_dash,
        Y_r_dash=Y_r_dash,
        Y_vvv_dash=Y_vvv_dash,
        Y_vvr_dash=Y_vvr_dash,
        Y_vrr_dash=Y_vrr_dash,
        Y_rrr_dash=Y_rrr_dash,
        N_v_dash=N_v_dash,
        N_r_dash=N_r_dash,
        N_vvv_dash=N_vvv_dash,
        N_vvr_dash=N_vvr_dash,
        N_vrr_dash=N_vrr_dash,
        N_rrr_dash=N_rrr_dash,
        R_0=0.5*basic_params.ρ*(basic_params.L_pp**2)*basic_params.d*R_0_dash
        )

coefficients_file = "./input/149_coefficient_sample.csv"
position_file = "./input/df_position.csv"
velocity_file = "./input/df_velocity.csv"
actuator_file = "./input/df_actuator.csv"

def get_KVLCC2_L7model_basic_params(
    ρ = 1025.0,  # 海水密度
    L_pp = 7.00,  # 船長Lpp[m]
    B = 1.27,  # 船幅[m]
    d = 0.46,  # 喫水[m]
    nabla = 3.27,  # 排水量[m^3]
    x_G = 0.25,  # 重心位置[m]
    C_b = 0.810,  # 方形係数[-]
    D_p = 0.216,  # プロペラ直径[m]
    H_R = 0.345,  # 舵高さ[m]
    A_R = 0.0539,  # 舵断面積[m^2]
    t_P = 0.220,  # 推力減少率
    w_P0 = 0.40,  # 有効伴流率
    m_x_dash = 0.022,  # 付加質量x(無次元)
    m_y_dash = 0.223,  # 付加質量y(無次元)
    J_z_dash = 0.011,  # 付加質量Izz(無次元)
    t_R = 0.387,  # 操縦抵抗減少率
    x_R_dash = -0.500,  # 舵の相対位置
    a_H = 0.312,  # 舵力増加係数
    x_H_dash = -0.464,  # 舵力増分作用位置
    γ_R_minus = 0.395,  # 整流係数
    γ_R_plus = 0.640,  # 整流係数
    l_r_dash = -0.710,  # 船長に対する舵位置
    x_P_dash = -0.480,  # 船長に対するプロペラ位置
    ϵ = 1.09,  # プロペラ・舵位置伴流係数比
    κ = 0.50,  # 修正係数
    f_α = 2.747,  # 直圧力勾配係数
    ):
    return set_basic_params(ρ, L_pp, B, d, nabla, x_G, C_b, D_p, H_R, A_R, t_P, w_P0,
                            m_x_dash, m_y_dash, J_z_dash, t_R, x_R_dash, a_H, x_H_dash,
                            γ_R_minus, γ_R_plus, l_r_dash, x_P_dash, ϵ, κ, f_α)

def get_KVLCC2_L7model_mmg_params(
    k_0 = 0.2931,
    k_1 = -0.2753,
    k_2 = -0.1385,
    R_0_dash = 0.022,
    X_vv_dash = -0.040,
    X_vr_dash = 0.002,
    X_rr_dash = 0.011,
    X_vvvv_dash = 0.771,
    Y_v_dash = -0.315,
    Y_r_dash = 0.083,
    Y_vvv_dash = -1.607,
    Y_vvr_dash = 0.379,
    Y_vrr_dash = -0.391,
    Y_rrr_dash = 0.008,
    N_v_dash = -0.137,
    N_r_dash = -0.049,
    N_vvv_dash = -0.030,
    N_vvr_dash = -0.294,
    N_vrr_dash = 0.055,
    N_rrr_dash = -0.013,
    basic_params = get_KVLCC2_L7model_basic_params()
    ):
    return set_mmg_params(k_0, k_1, k_2,
        R_0_dash, X_vv_dash, X_vr_dash, X_rr_dash, X_vvvv_dash,
        Y_v_dash, Y_r_dash, Y_vvv_dash, Y_vvr_dash, Y_vrr_dash, Y_rrr_dash,
        N_v_dash, N_r_dash, N_vvv_dash, N_vvr_dash, N_vrr_dash, N_rrr_dash, basic_params)

def get_default_mpc_params(
    n_horizon = 100,
    t_step = 1,
    n_robust = 1,
    store_full_solution = True,
    δ_set = 1e-3,
    n_p_set = 1e-3,
    δ_max = 45.0*np.pi/180.0,
    δ_min = -45.0*np.pi/180.0,
    n_p_max = 50.0,
    n_p_min = 0.0,
    u_max = 15.0,
    u_min = 0.0,
    v_max = 10.0,
    v_min = -10.0,
    control_duration = 100,
    model_type = 'continuous'
    ):
    return set_mpc_params(n_horizon, t_step, n_robust, store_full_solution, 
        δ_set, n_p_set, δ_max, δ_min, n_p_max, n_p_min, 
        u_max, u_min, v_max, v_min, control_duration, model_type)

def get_149ship_basic_params(
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
    ):
    return set_basic_params(ρ, L_pp, B, d, nabla, x_G, C_b, D_p, H_R, A_R, t_P, w_P0,
                            m_x_dash, m_y_dash, J_z_dash, t_R, x_R_dash, a_H, x_H_dash,
                            γ_R_minus, γ_R_plus, l_r_dash, x_P_dash, ϵ, κ, f_α)

