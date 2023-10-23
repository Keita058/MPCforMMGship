import os
import numpy as np
import pandas as pd
import do_mpc
from do_mpc.data import save_results


class ModelPredictiveControl:
    def __init__(self, basic_params, mmg_params, mpc_params, input_files):
        self.basic_params = basic_params
        self.mmg_params = mmg_params
        self.mpc_params = mpc_params
        self.input_files = input_files
    
    def read_pos_data(self,file_name):
        df=pd.read_csv(file_name)
        psi,xc,yc,x1,y1,x2,y2=df['psi'],df['xc'],df['yc'],df['x1'],df['y1'],df['x2'],df['y2']
        return psi,xc,yc,x1,y1,x2,y2

    #制御入力を読み込む関数
    def read_act_data(self,file_name):
        df=pd.read_csv(file_name)
        npm,delta=df['npm'],df['delta']
        return npm,delta
    #速度を読み込む関数
    def read_vel_data(self,file_name):
        df=pd.read_csv(file_name)
        u,v,r=df['u'],df['v'],df['r']
        return u,v,r

    def set_variables(self, model):
        #MPCの変数の名前,タイプの設定
        self.δ_set = model.set_variable(var_type='_u', var_name='δ_set', shape=(1,1)) #指示舵角
        self.n_p_set=model.set_variable(var_type='_u',var_name='n_p_set',shape=(1,1)) #指示プロペラ回転数

        self.xc = model.set_variable(var_type='_x', var_name='xc', shape=(1,1)) #船の中心x座標
        self.yc= model.set_variable(var_type='_x', var_name='yc', shape=(1,1)) #船の中心y座標
        self.x1 = model.set_variable(var_type='_x', var_name='x1', shape=(1,1)) #船のバウx座標
        self.y1= model.set_variable(var_type='_x', var_name='y1', shape=(1,1)) #船のバウy座標
        self.x2 = model.set_variable(var_type='_x', var_name='x2', shape=(1,1)) #船のスタンx座標
        self.y2= model.set_variable(var_type='_x', var_name='y2', shape=(1,1)) #船のスタンy座標
        self.ψ = model.set_variable(var_type='_x', var_name='ψ', shape=(1,1)) #船首角(x軸正の方向となす角)
        self.u = model.set_variable(var_type='_x', var_name='u', shape=(1,1)) #前後方向速度
        self.v = model.set_variable(var_type='_x', var_name='v', shape=(1,1)) #横方向速度
        self.r = model.set_variable(var_type='_x', var_name='r', shape=(1,1)) #回頭角速度
        self.δ=model.set_variable(var_type='_x',var_name='δ',shape=(1,1)) #舵角
        self.n_p=model.set_variable(var_type='_x',var_name='n_p',shape=(1,1)) #プロペラ回転数

        self.x1_ref=model.set_variable(var_type='_tvp',var_name='x1_ref',shape=(1,1)) #目標軌道のx座標
        self.y1_ref=model.set_variable(var_type='_tvp',var_name='y1_ref',shape=(1,1)) #目標軌道のy座標
        self.x2_ref=model.set_variable(var_type='_tvp',var_name='x2_ref',shape=(1,1)) #目標軌道のx座標
        self. y2_ref=model.set_variable(var_type='_tvp',var_name='y2_ref',shape=(1,1)) #目標軌道のy座標

    def set_model_for_MPC(self, model, MMG):
        #MPCに使う運動方程式の設定
        u=self.u
        v=self.v
        r=self.r
        ψ=self.ψ
        δ_set=self.δ_set
        n_p_set=self.n_p_set
        δ=self.δ
        n_p=self.n_p
        m=self.basic_params.m
        m_y=self.basic_params.m_y
        m_x=self.basic_params.m_x
        I_zG=self.basic_params.I_zG
        J_z=self.basic_params.J_z
        model.set_rhs('u', (MMG.X_H(u,v,r)+MMG.X_R(u,v,r,δ,n_p)+MMG.X_P(u,v,δ,n_p)+(m+m_y)*v*r)/(m+m_x))
        model.set_rhs('v', (MMG.Y_H(u,v,r)+MMG.Y_R(u,v,r,δ,n_p)-(m+m_x)*u*r)/(m+m_y))
        model.set_rhs('r', (MMG.N_H(u,v,r)+MMG.N_R(u,v,r,δ,n_p))/(I_zG+J_z))
        model.set_rhs('xc', u * np.cos(ψ) - v * np.sin(ψ))
        model.set_rhs('yc', u * np.sin(ψ) + v * np.cos(ψ))
        model.set_rhs('x1', u * np.cos(ψ) - v * np.sin(ψ))
        model.set_rhs('y1', u * np.sin(ψ) + v * np.cos(ψ))
        model.set_rhs('x2', u * np.cos(ψ) - v * np.sin(ψ))
        model.set_rhs('y2', u * np.sin(ψ) + v * np.cos(ψ))
        model.set_rhs('ψ', r)
        model.set_rhs('δ',(δ_set-δ))
        model.set_rhs('n_p',(n_p_set-n_p))

    def set_evaluation_func(self, model, mpc):
        lterm=(model.x['x1']-model.tvp['x1_ref'])**2+(model.x['y1']-model.tvp['y1_ref'])**2+ \
            (model.x['x2']-model.tvp['x2_ref'])**2+(model.x['y2']-model.tvp['y2_ref'])**2
        mterm=lterm
        mpc.set_objective(mterm=mterm,lterm=lterm)

        mpc.set_rterm(
            δ_set = self.mpc_params.δ_set,
            n_p_set = self.mpc_params.n_p_set
        )
    
    def set_mpc_bounds(self, mpc):
        mpc.bounds['lower','_u', 'δ_set'] = self.mpc_params.δ_min
        mpc.bounds['upper','_u', 'δ_set'] = self.mpc_params.δ_max
        mpc.bounds['lower','_x','u'] = self.mpc_params.u_min
        mpc.bounds['upper','_x','u'] = self.mpc_params.u_max
        mpc.bounds['lower','_x','v'] = self.mpc_params.v_min
        mpc.bounds['upper','_x','v'] = self.mpc_params.v_max
        mpc.bounds['lower','_u','n_p_set'] = self.mpc_params.n_p_min
        mpc.bounds['upper','_u','n_p_set'] = self.mpc_params.n_p_max
    

    def main(self,MMG):
        model_type = self.mpc_params.model_type
        model = do_mpc.model.Model(model_type)

        self.set_variables(model)

        self.set_model_for_MPC(model, MMG)

        model.setup()

        mpc = do_mpc.controller.MPC(model)

        n_horizon = self.mpc_params.n_horizon
        t_step = self.mpc_params.t_step
        n_robust = self.mpc_params.n_robust
        store_full_solution = self.mpc_params.store_full_solution
        control_duration = self.mpc_params.control_duration

        setup_mpc = {
            'n_horizon': n_horizon,
            't_step': t_step,
            'n_robust': n_robust,
            'store_full_solution': store_full_solution,
        }

        mpc.set_param(**setup_mpc)

        self.set_evaluation_func(model, mpc)

        self.set_mpc_bounds(mpc)

        psi0_ref,xc0_ref,yc0_ref,x10_ref,y10_ref,x20_ref,y20_ref=self.read_pos_data(self.input_files.position_file)
        u_ref,v_ref,r_ref=self.read_vel_data(self.input_files.velocity_file)
        npm_ref,delta_ref=self.read_act_data(self.input_files.actuator_file)

        n_steps = len(psi0_ref)


        tvp_template = mpc.get_tvp_template()
        def tvp_fun(t_now):
            for k in range(n_horizon):
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
        simulator.set_param(t_step = self.mpc_params.t_step)
        tvp_sim_template=simulator.get_tvp_template()
        def tvp_sim_fun(t_now):
            return tvp_sim_template
        simulator.set_tvp_fun(tvp_sim_fun)
        simulator.setup()

        x0 = np.array([xc0_ref[0],yc0_ref[0],x10_ref[0],y10_ref[0],x20_ref[0],y20_ref[0],psi0_ref[0],u_ref[0],v_ref[0],r_ref[0],delta_ref[0],npm_ref[0]]).reshape(-1,1)
        simulator.x0 = x0
        mpc.x0 = x0
        estimator.x0=x0
        mpc.set_initial_guess()

        u0 = mpc.make_step(x0)
        u0 = mpc.make_step(x0)

        simulator.reset_history()
        simulator.x0 = x0
        mpc.x0=x0
        mpc.reset_history()

        for _ in range(min(control_duration,n_steps)):
            u0 = mpc.make_step(x0)
            y_next=simulator.make_step(u0)
            x0=estimator.make_step(y_next)
        
        # remove past results
        os.remove("./results/results.pkl")

        # save present result
        save_results([mpc, simulator])