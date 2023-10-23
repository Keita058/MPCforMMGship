import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

blue='#1f77b4'
yellow='#ff7f0e'


#outputに出力されたファイルを読み込み、グラフを作成するクラス
#軌道、誤差、各状態量、制御量

class Make_Graphs:
    def __init__(self,output_dir_path):
        self.output_dir_path=output_dir_path
        self.output_csv_path=output_dir_path+'/output.csv'
        self.save_dir_path=output_dir_path+'/images'
        os.makedirs(self.save_dir_path,exist_ok=True)

        self.df=pd.read_csv(self.output_csv_path)

        self.set_unit()

    def set_unit(self):
        unit_dict={}
        for col in self.df.columns:
            if 'y' in col or 'x' in col or 'error' in col:
                unit_dict[col]='[m]'
            elif 'psi' in col or 'delta' in col:
                unit_dict[col]='[deg]'
            elif 'u' in col or 'v' in col or 'U' in col:
                unit_dict[col]='[m/s]'
            elif col=='r' or col=='r_ref':
                unit_dict[col]='[rad/s]'
            elif 'npm' in col:
                unit_dict[col]='[rps]'
            else:
                unit_dict[col]='[s]'
        self.unit_dict=unit_dict

    def trajectory_image(self, image_scale=16, compare=False):
        xc_ref=self.df['xc_ref']
        yc_ref=self.df['yc_ref']
        xc=self.df['xc']
        yc=self.df['yc']

        min_X,max_X=min(min(xc_ref),min(xc)),max(max(xc_ref),max(xc))
        min_Y,max_Y=min(min(yc_ref),min(yc)),max(max(yc_ref),max(yc))
        gcd_XY=np.gcd(int(max_X-min_X),int(max_Y-min_Y))
        figsize_X,figsize_Y=(max_X-min_X)/gcd_XY,(max_Y-min_Y)/gcd_XY
        scale=image_scale/max(figsize_X,figsize_Y)
        fig=plt.figure(figsize=(figsize_X*scale,figsize_Y*scale))

        if compare:
            plt.plot(xc_ref,yc_ref,color=yellow,label='model ship',ls='--')
            plt.plot(xc,yc,color=blue,label='MPC')

        else:
            plt.plot(xc,yc,color=blue,label='MPC')

        X_plus,Y_plus=(max_X-min_X)/20,(max_Y-min_Y)/20
        plt.xlim([min_X-X_plus,max_X+X_plus])
        plt.ylim([min_Y-Y_plus,max_Y+Y_plus])
        dist_Y=((max_Y-min_Y+Y_plus*2+99)//100)*10
        dist_X=((max_X-min_X+X_plus*2+99)//100)*10
        plt.xticks(list(filter(lambda x: x%dist_X==0, np.arange(int(min_X-X_plus),int(max_X+X_plus)))))
        plt.yticks(list(filter(lambda x: x%dist_Y==0, np.arange(int(min_Y-Y_plus),int(max_Y+Y_plus)))))
        plt.xlabel('X[m]',fontsize=24)
        plt.ylabel('Y[m]',fontsize=24)
        plt.tick_params(labelsize=24)
        plt.legend(fontsize=18)
        plt.tight_layout()
        if compare:
            fig.savefig(self.save_dir_path+'/trajectory_compare.png')
        else:
            fig.savefig(self.save_dir_path+'/trajectory.png')
        plt.close()

    def error_image(self):
        fig=plt.figure(figsize=(16,4))
        plt.plot(self.df['time'],self.df['error'],color=blue)
        plt.xlabel('time[s]',fontsize=24)
        plt.ylabel('error[m]',fontsize=24)
        plt.minorticks_on()
        plt.grid(which='both',color='gray',ls='--',axis="y")
        plt.tick_params(labelsize=24)
        plt.tight_layout()
        fig.savefig(self.save_dir_path+'/error.png')
        plt.close()

    def comparison_image(self,state_name):
        ref_state=state_name+'_ref'
        fig=plt.figure(figsize=(16,4))
        plt.plot(self.df['time'],self.df[state_name],color=blue,label='MPC')
        plt.plot(self.df['time'],self.df[ref_state],color=yellow,ls='--',label='model ship')
        plt.xlabel('time[s]',fontsize=24)
        plt.ylabel(state_name+self.unit_dict[state_name],fontsize=24)
        plt.minorticks_on()
        plt.grid(which='both',color='gray',ls='--',axis="y")
        plt.tick_params(labelsize=24)
        plt.legend(fontsize=18)
        plt.tight_layout()
        fig.savefig(self.save_dir_path+'/'+state_name+'.png')
        plt.close()

    def main(self):
            Graphs=Make_Graphs(self.output_dir_path)
            Graphs.trajectory_image(compare=True)
            Graphs.trajectory_image(compare=False)
            Graphs.error_image()
            state_cols=self.df.columns
            n_state=list()
            for state_col in state_cols:
                if '_' not in state_col and state_col!='time' and state_col!='error':
                    n_state.append(state_col)
            state_cols=n_state
            for state_col in state_cols:
                Graphs.comparison_image(state_col)