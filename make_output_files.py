import numpy as np
import pandas as pd
import os

class make_outputs:
    def __init__(self,results, mmg_params, mpc_params, input_files, dirname, sample_num):
        self.results=results
        self.input_files=input_files
        self.mpc_params=mpc_params
        self.mmg_params=mmg_params
        self.dirname=dirname
        self.sample_num=sample_num

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
    
    def speed_magnitude(self, u, v):
        U=[]
        for i in range(len(u)):
            U.append(np.sqrt(u[i]**2+v[i]**2))
        return U

    def make_error(self, x_ref, y_ref, x, y):
        error=[]
        for i in range(min(len(x_ref),len(x))):
            error.append(np.sqrt((x_ref[i]-x[i])**2+(y_ref[i]-y[i])**2))
        return error
    
    def make_mmg_dataframe(self, mmg_params):
        df=pd.DataFrame({
            'k_0':[mmg_params.k_0],
            'k_1':[mmg_params.k_1],
            'k_2':[mmg_params.k_2],
            'R_0_dash':[mmg_params.R_0_dash],
            'X_vv_dash':[mmg_params.X_vv_dash],
            'X_vr_dash':[mmg_params.X_vr_dash],
            'X_rr_dash':[mmg_params.X_rr_dash],
            'X_vvvv_dash':[mmg_params.X_vvvv_dash],
            'Y_v_dash':[mmg_params.Y_v_dash],
            'Y_r_dash':[mmg_params.Y_r_dash],
            'Y_vvv_dash':[mmg_params.Y_vvv_dash],
            'Y_vvr_dash':[mmg_params.Y_vvr_dash],
            'Y_vrr_dash':[mmg_params.Y_vrr_dash],
            'Y_rrr_dash':[mmg_params.Y_rrr_dash],
            'N_v_dash':[mmg_params.N_v_dash],
            'N_r_dash':[mmg_params.N_r_dash],
            'N_vvv_dash':[mmg_params.N_vvv_dash],
            'N_vvr_dash':[mmg_params.N_vvr_dash],
            'N_vrr_dash':[mmg_params.N_vrr_dash],
            'N_rrr_dash':[mmg_params.N_rrr_dash],
        })
        return df

    def make_dataframe_from_results(self, results, mpc_params, input_files):
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

        psi_ref,xc_ref,yc_ref,x1_ref,y1_ref,x2_ref,y2_ref=self.read_pos_data(input_files.position_file)
        u_ref,v_ref,r_ref=self.read_vel_data(input_files.velocity_file)
        npm_ref,delta_ref=self.read_act_data(input_files.actuator_file)

        control_duration=mpc_params.control_duration
        n_steps=len(psi_ref)

        U=self.speed_magnitude(u,v)
        U_ref=self.speed_magnitude(u_ref,v_ref)
        error=self.make_error(xc_ref,yc_ref,xc,yc)

        length=min(control_duration, n_steps)

        t=np.linspace(0,length-1,length)

        df=pd.DataFrame({'time':t,
            'xc':xc,
            'xc_ref':xc_ref[:length],
            'yc':yc,
            'yc_ref':yc_ref[:length],
            'x1':x1,
            'x1_ref':x1_ref[:length],
            'y1':y1,
            'y1_ref':y1_ref[:length],
            'x2':x2,
            'x2_ref':x2_ref[:length],
            'y2':y2,
            'y2_ref':y2_ref[:length],
            'psi':psi,
            'psi_ref':psi_ref[:length],
            'U':U,
            'U_ref':U_ref[:length],
            'u':u,
            'u_ref':u_ref[:length],
            'v':v,
            'v_ref':v_ref[:length],
            'r':r,
            'r_ref':r_ref[:length],
            'delta':delta,
            'delta_ref':delta_ref[:length],
            'delta_in':delta_in,
            'npm':n_P,
            'npm_ref':npm_ref[:length],
            'npm_in':np_in,
            'error':error
            })
        return df

    def main(self):
        if self.sample_num is not None:
            self.dirname=self.dirname+'/sample'+str(self.sample_num)
        
        os.makedirs(self.dirname)

        output_df=self.make_dataframe_from_results(self.results, self.mpc_params, self.input_files)
        output_df.to_csv(self.dirname+'/output.csv',index=True)

        df_position=pd.read_csv(self.input_files.position_file)
        df_position.to_csv(self.dirname+'/input_position.csv',index=True)

        df_velocity=pd.read_csv(self.input_files.velocity_file)
        df_velocity.to_csv(self.dirname+'/input_velocity.csv',index=True)

        df_actuator=pd.read_csv(self.input_files.actuator_file)
        df_actuator.to_csv(self.dirname+'/input_actuator.csv',index=True)

        df_mmg_coef = self.make_mmg_dataframe(self.mmg_params)
        df_mmg_coef.to_csv(self.dirname+'/mmg_coefficients.csv',index=True)