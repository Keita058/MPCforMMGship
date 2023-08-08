import csv
import os
import numpy as np
import pandas as pd
import do_mpc
from casadi import vertcat
from do_mpc.data import save_results, load_results

blue='#1f77b4'
yellow='#ff7f0e'

#Certain parameters
ρ = 1025.0  # 海水密度

L_pp = 7.00  # 船長Lpp[m]
B = 1.27  # 船幅[m]
d = 0.46  # 喫水[m]
nabla = 3.27  # 排水量[m^3]
x_G = 0.25  # 重心位置[m]
# C_b = 0.810  # 方形係数[-]
D_p = 0.216  # プロペラ直径[m]
H_R = 0.345  # 舵高さ[m]
A_R = 0.0539  # 舵断面積[m^2]

t_P = 0.220  # 推力減少率
w_P0 = 0.40  # 有効伴流率
m_x_dash = 0.022  # 付加質量x(無次元)
m_y_dash = 0.223  # 付加質量y(無次元)
J_z_dash = 0.011  # 付加質量Izz(無次元)
t_R = 0.387  # 操縦抵抗減少率
x_R_dash = -0.500  # 舵の相対位置
a_H = 0.312  # 舵力増加係数
x_H_dash = -0.464  # 舵力増分作用位置
γ_R_minus = 0.395  # 整流係数
γ_R_plus = 0.640  # 整流係数
l_r_dash = -0.710  # 船長に対する舵位置
x_P_dash = -0.480  # 船長に対するプロペラ位置
ϵ = 1.09  # プロペラ・舵位置伴流係数比
κ = 0.50  # 修正係数
f_α = 2.747  # 直圧力勾配係数

m=ρ * nabla  # 質量(無次元化)[kg]
I_zG=ρ * nabla * ((0.25 * L_pp) ** 2)  # 慣性モーメント[-]
η=D_p / H_R # プロペラ直径に対する舵高さ(t_Dp/H)
m_x=(0.5 * ρ * (L_pp ** 2) * d) * m_x_dash  # 付加質量x(無次元)
m_y=(0.5 * ρ * (L_pp ** 2) * d) * m_y_dash  # 付加質量y(無次元)
J_z=(0.5 * ρ * (L_pp ** 4) * d) * J_z_dash  # 付加質量Izz(無次元)
x_H=x_H_dash * L_pp  # 舵力増分作用位置
x_R=x_R_dash*L_pp

k_0 = 0.2931
k_1 = -0.2753
k_2 = -0.1385

def read_pos_data(file_name):
    df=pd.read_csv(file_name)
    psi,xc,yc,x1,y1,x2,y2=df['psi'],df['xc'],df['yc'],df['x1'],df['y1'],df['x2'],df['y2']
    return psi,xc,yc,x1,y1,x2,y2

def read_act_data(file_name):
    df=pd.read_csv(file_name)
    npm,delta=df['npm'],df['delta']
    return npm,delta

def read_vel_data(file_name):
    df=pd.read_csv(file_name)
    u,v,r=df['u'],df['v'],df['r']
    return u,v,r

#運動方程式を有次元値で扱っている
def X_H(u,v,r):
    U=np.sqrt(u**2+v**2)
    v_dash=v/U
    r_dash=r*L_pp/U
    return 0.5*ρ*L_pp*d*U**2*X_H_dash(v_dash,r_dash)

def Y_H(u,v,r):
    U=np.sqrt(u**2+v**2)
    v_dash=v/U
    r_dash=r*L_pp/U
    return 0.5*ρ*L_pp*d*U**2*Y_H_dash(v_dash,r_dash)

def N_H(u,v,r):
    U=np.sqrt(u**2+v**2)
    v_dash=v/U
    r_dash=r*L_pp/U
    return 0.5*ρ*L_pp**2*d*U**2*N_H_dash(v_dash,r_dash)

#斜航角ではなく横方向の速度の無次元化成分を使う？
def X_H_dash(β,r_dash):
    return -R_0_dash+X_vv_dash*β**2+X_vr_dash*β*r_dash+X_rr_dash*r_dash**2+X_vvvv_dash*β**4

def Y_H_dash(β,r_dash):
    return Y_v_dash*β+Y_r_dash*r_dash+Y_vvr_dash*β**2*r_dash+Y_vrr_dash*β*r_dash**2+Y_vvv_dash*β**3+Y_rrr_dash*r_dash**3

def N_H_dash(β,r_dash):
    return N_v_dash*β+N_r_dash*r_dash+N_vvr_dash*β**2*r_dash+N_vrr_dash*β*r_dash**2+N_vvv_dash*β**3+N_rrr_dash*r_dash**3

def X_R(u,v,r,δ,n_p):
    return -(1-t_R)*F_N(u,v,r,δ,n_p)*np.sin(δ)

def Y_R(u,v,r,δ,n_p):
    return -(1+a_H)*F_N(u,v,r,δ,n_p)*np.cos(δ)

def N_R(u,v,r,δ,n_p):
    return -(x_R+a_H*x_H)*F_N(u,v,r,δ,n_p)*np.cos(δ)

def F_N(u,v,r,δ,n_p):
    u_p=(1-w_P0)*u
    u_inf=u_p*np.sqrt(1+(8*K_T(u,n_p))/(np.pi*J(u,n_p)**2))
    k_x=κ*ϵ
    delta_u=u_inf-u_p
    u_RP=u_p+k_x*delta_u
    u_R=np.sqrt(η*u_RP**2+(1-η)*u_p**2)
    U=np.sqrt(u**2+v**2)
    β=np.arctan(-v/U)
    r_dash=r*L_pp/U
    v_R=U*γ_R_plus*(β-l_r_dash*r_dash)
    U_R=np.sqrt(u_R**2+v_R**2)
    α_R=δ-np.arctan(v_R/u_R)
    return 0.5*ρ*A_R*U_R**2*f_α*np.sin(α_R)

def X_P(u,v,δ,n_p):
    return (1-t_P)*T_P(u,n_p)

def K_T(u,n_p):
    return k_0+k_1*J(u,n_p)+k_2*J(u,n_p)**2

def J(u,n_p):
    return u*(1-w_P0)/(n_p*D_p)

def T_P(u,n_p):
    return K_T(u,n_p)*ρ*n_p**2*D_p**4


control_duration=100
dirname='./output'
n_horizon=100

os.makedirs(dirname,exist_ok=True)


R_0_dash = 0.022
X_vv_dash = -0.040
X_vr_dash = 0.002
X_rr_dash = 0.011
X_vvvv_dash = 0.771
Y_v_dash = -0.315
Y_r_dash = 0.083
Y_vvv_dash = -1.607
Y_vvr_dash = 0.379
Y_vrr_dash = -0.391
Y_rrr_dash = 0.008
N_v_dash = -0.137
N_r_dash = -0.049
N_vvv_dash = -0.030
N_vvr_dash = -0.294
N_vrr_dash = 0.055
N_rrr_dash = -0.013
R0=0.5*ρ*(L_pp**2)*d*R_0_dash #船体抵抗

model_type = 'continuous' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

δ_set = model.set_variable(var_type='_u', var_name='δ_set', shape=(1,1)) #指示舵角
n_p_set=model.set_variable(var_type='_u',var_name='n_p_set',shape=(1,1)) #指示プロペラ回転数

xc = model.set_variable(var_type='_x', var_name='xc', shape=(1,1)) #船の中心x座標
yc= model.set_variable(var_type='_x', var_name='yc', shape=(1,1)) #船の中心y座標
x1 = model.set_variable(var_type='_x', var_name='x1', shape=(1,1)) #船のバウx座標
y1= model.set_variable(var_type='_x', var_name='y1', shape=(1,1)) #船のバウy座標
x2 = model.set_variable(var_type='_x', var_name='x2', shape=(1,1)) #船のスタンx座標
y2= model.set_variable(var_type='_x', var_name='y2', shape=(1,1)) #船のスタンy座標
ψ = model.set_variable(var_type='_x', var_name='ψ', shape=(1,1)) #船首角(x軸正の方向となす角)
u = model.set_variable(var_type='_x', var_name='u', shape=(1,1)) #前後方向速度
v = model.set_variable(var_type='_x', var_name='v', shape=(1,1)) #横方向速度
r = model.set_variable(var_type='_x', var_name='r', shape=(1,1)) #回頭角速度
δ=model.set_variable(var_type='_x',var_name='δ',shape=(1,1)) #舵角
n_p=model.set_variable(var_type='_x',var_name='n_p',shape=(1,1)) #プロペラ回転数

x1_ref=model.set_variable(var_type='_tvp',var_name='x1_ref',shape=(1,1)) #目標軌道のx座標
y1_ref=model.set_variable(var_type='_tvp',var_name='y1_ref',shape=(1,1)) #目標軌道のy座標
x2_ref=model.set_variable(var_type='_tvp',var_name='x2_ref',shape=(1,1)) #目標軌道のx座標
y2_ref=model.set_variable(var_type='_tvp',var_name='y2_ref',shape=(1,1)) #目標軌道のy座標

model.set_rhs('u', (X_H(u,v,r)+X_R(u,v,r,δ,n_p)+X_P(u,v,δ,n_p)+(m+m_y)*v*r)/(m+m_x))
model.set_rhs('v', (Y_H(u,v,r)+Y_R(u,v,r,δ,n_p)-(m+m_x)*u*r)/(m+m_y))
model.set_rhs('r', (N_H(u,v,r)+N_R(u,v,r,δ,n_p))/(I_zG+J_z))
model.set_rhs('xc', u * np.cos(ψ) - v * np.sin(ψ))
model.set_rhs('yc', u * np.sin(ψ) + v * np.cos(ψ))
model.set_rhs('x1', u * np.cos(ψ) - v * np.sin(ψ))
model.set_rhs('y1', u * np.sin(ψ) + v * np.cos(ψ))
model.set_rhs('x2', u * np.cos(ψ) - v * np.sin(ψ))
model.set_rhs('y2', u * np.sin(ψ) + v * np.cos(ψ))
model.set_rhs('ψ', r)
model.set_rhs('δ',(δ_set-δ))
model.set_rhs('n_p',(n_p_set-n_p))

model.setup()

mpc = do_mpc.controller.MPC(model)

n_horizon =n_horizon
t_step = 1
n_robust = 1 # default to 0
store_full_solution = True

setup_mpc = {
    'n_horizon': n_horizon,
    't_step': t_step,
    'n_robust': n_robust,
    'store_full_solution': store_full_solution,
}

mpc.set_param(**setup_mpc)

lterm=(model.x['x1']-model.tvp['x1_ref'])**2+(model.x['y1']-model.tvp['y1_ref'])**2+ \
    (model.x['x2']-model.tvp['x2_ref'])**2+(model.x['y2']-model.tvp['y2_ref'])**2
mterm=lterm
mpc.set_objective(mterm=mterm,lterm=lterm)

mpc.set_rterm(
    δ_set = 1e-3,
    n_p_set=1e-3
)

mpc.bounds['lower','_u', 'δ_set'] = - 45 * np.pi / 180
mpc.bounds['upper','_u', 'δ_set'] = 45 * np.pi / 180
mpc.bounds['lower','_x','u']=0.0
mpc.bounds['upper','_x','u']=15.0
mpc.bounds['lower','_x','v']=-5.0
mpc.bounds['upper','_x','v']=5.0
mpc.bounds['lower','_u','n_p_set']=0.0
mpc.bounds['upper','_u','n_p_set']=30.0

psi0_ref,xc0_ref,yc0_ref,x10_ref,y10_ref,x20_ref,y20_ref=read_pos_data('./input/df_position.csv')
n_steps=len(xc0_ref)

tvp_template=mpc.get_tvp_template()

def tvp_fun(t_now):
    for k in range(mpc.n_horizon):
        if int(t_now)+k<n_steps:
            tvp_template['_tvp',k,'x1_ref']=x10_ref[int(t_now)+k]
            tvp_template['_tvp',k,'y1_ref']=y10_ref[int(t_now)+k]
            tvp_template['_tvp',k,'x2_ref']=x20_ref[int(t_now)+k]
            tvp_template['_tvp',k,'y2_ref']=y20_ref[int(t_now)+k]
        else:
            tvp_template['_tvp',k,'x1_ref']=x10_ref[n_steps-1]
            tvp_template['_tvp',k,'y1_ref']=y10_ref[n_steps-1]
            tvp_template['_tvp',k,'x2_ref']=x20_ref[n_steps-1]
            tvp_template['_tvp',k,'y2_ref']=y20_ref[n_steps-1]
    return tvp_template
mpc.set_tvp_fun(tvp_fun)

mpc.setup()

estimator=do_mpc.estimator.StateFeedback(model)
simulator = do_mpc.simulator.Simulator(model)

simulator.set_param(t_step = 1)

tvp_sim_template=simulator.get_tvp_template()

def tvp_sim_fun(t_now):
    return tvp_sim_template
simulator.set_tvp_fun(tvp_sim_fun)

simulator.setup()

npm_ref,delta_ref=read_act_data('./input/df_actuator.csv')
u_ref,v_ref,r_ref=read_vel_data('./input/df_velocities.csv')

x0 = np.array([xc0_ref[0],yc0_ref[0],x10_ref[0],y10_ref[0],x20_ref[0],y20_ref[0],psi0_ref[0],u_ref[0],v_ref[0],r_ref[0],delta_ref[0],npm_ref[0]]).reshape(-1,1)
simulator.x0 = x0
mpc.x0 = x0
estimator.x0=x0
mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

u0 = mpc.make_step(x0)

u0 = mpc.make_step(x0)

simulator.reset_history()
simulator.x0 = x0
mpc.x0=x0
mpc.reset_history()

for i in range(min(control_duration,n_steps)):
    u0 = mpc.make_step(x0)
    y_next=simulator.make_step(u0)
    x0=estimator.make_step(y_next)

# remove past results
import os
os.remove("./results/results.pkl")

# save present result
save_results([mpc, simulator])
results = load_results('./results/results.pkl')

x1_ref=list(results['mpc']['_tvp','x1_ref'].reshape(-1))
y1_ref=list(results['mpc']['_tvp','y1_ref'].reshape(-1))
x2_ref=list(results['mpc']['_tvp','x2_ref'].reshape(-1))
y2_ref=list(results['mpc']['_tvp','y2_ref'].reshape(-1))
xc=list(results['mpc']['_x','xc'].reshape(-1))
yc=list(results['mpc']['_x','yc'].reshape(-1))
x1=list(results['mpc']['_x','x1'].reshape(-1))
y1=list(results['mpc']['_x','y1'].reshape(-1))
x2=list(results['mpc']['_x','x2'].reshape(-1))
y2=list(results['mpc']['_x','y2'].reshape(-1))
psi=list(results['simulator']['_x','ψ'].reshape(-1))
u=list(results['simulator']['_x','u'].reshape(-1))
v=list(results['simulator']['_x','v'].reshape(-1))
r=list(results['simulator']['_x','r'].reshape(-1))
delta=list(results['simulator']['_x','δ'].reshape(-1))
n_P=list(results['simulator']['_x','n_p'].reshape(-1))
delta_in=list(results['simulator']['_u','δ_set'].reshape(-1))
np_in=list(results['simulator']['_u','n_p_set'].reshape(-1))


U=[]
U_ref=[]
for i in range(len(u)):
    U.append(np.sqrt(u[i]**2+v[i]**2))
    U_ref.append(np.sqrt(u_ref[i]**2+v_ref[i]**2))

error=[]
for i in range(min(control_duration,n_steps)):
    dist=(xc[i]-xc0_ref[i])**2+(yc[i]-yc0_ref[i])**2
    error.append(np.sqrt(dist))

import pandas as pd

t=np.linspace(0,min(control_duration,n_steps-1),min(control_duration,n_steps))
df=pd.DataFrame({'time':t,
    'xc':xc,
    'xc_ref':xc0_ref[:min(control_duration,n_steps)],
    'yc':yc,
    'yc_ref':yc0_ref[:min(control_duration,n_steps)],
    'x1':x1,
    'x1_ref':x1_ref,
    'y1':y1,
    'y1_ref':y1_ref,
    'x2':x2,
    'x2_ref':x2_ref,
    'y2':y2,
    'y2_ref':y2_ref,
    'psi':psi,
    'psi_ref':psi0_ref[:min(control_duration,n_steps)],
    'U':U,
    'U_ref':U_ref,
    'u':u,
    'u_ref':u_ref[:min(control_duration,n_steps)],
    'v':v,
    'v_ref':v_ref[:min(control_duration,n_steps)],
    'r':r,
    'r_ref':r_ref[:min(control_duration,n_steps)],
    'delta':delta,
    'delta_ref':delta_ref[:min(control_duration,n_steps)],
    'delta_in':delta_in,
    'npm':n_P,
    'npm_ref':npm_ref[:min(control_duration,n_steps)],
    'npm_in':np_in,
    'error':error
    })

df.to_csv(dirname+'/output.csv')
print("Program terminated successfully.")