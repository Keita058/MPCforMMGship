import get_parameters
import numpy as np


class MMG_model:
    def __init__(self,basic_params, mmg_params):
        self.ρ = basic_params.ρ
        self.L_pp = basic_params.L_pp
        self.B = basic_params.B
        self.d = basic_params.d
        self.nabla = basic_params.nabla
        self.x_G = basic_params.x_G
        self.C_b = basic_params.C_b
        self.D_p = basic_params.D_p
        self.H_R = basic_params.H_R
        self.A_R = basic_params.A_R
        self.t_P = basic_params.t_P
        self.w_P0 = basic_params.w_P0
        self.m_x_dash = basic_params.m_x_dash
        self.m_y_dash = basic_params.m_y_dash
        self.J_z_dash = basic_params.J_z_dash
        self.t_R = basic_params.t_R
        self.x_R_dash = basic_params.x_R_dash
        self.a_H = basic_params.a_H
        self.x_H_dash = basic_params.x_H_dash
        self.γ_R_minus = basic_params.γ_R_minus
        self.γ_R_plus = basic_params.γ_R_plus
        self.l_r_dash = basic_params.l_r_dash
        self.x_P_dash = basic_params.x_P_dash
        self.ϵ = basic_params.ϵ
        self.κ = basic_params.κ
        self.f_α = basic_params.f_α

        self.m=self.ρ * self.nabla 
        self.I_zG=self.ρ * self.nabla * ((0.25 * self.L_pp) ** 2)  
        self.η=self.D_p / self.H_R 
        self.m_x=(0.5 * self.ρ * (self.L_pp ** 2) * self.d) * self.m_x_dash  
        self.m_y=(0.5 * self.ρ * (self.L_pp ** 2) * self.d) * self.m_y_dash  
        self.J_z=(0.5 * self.ρ * (self.L_pp ** 4) * self.d) * self.J_z_dash  
        self.x_H=self.x_H_dash * self.L_pp  
        self.x_R=self.x_R_dash*self.L_pp

        self.k_0 = mmg_params.k_0
        self.k_1 = mmg_params.k_1
        self.k_2 = mmg_params.k_2
        self.R_0_dash = mmg_params.R_0_dash
        self.X_vv_dash = mmg_params.X_vv_dash
        self.X_vr_dash = mmg_params.X_vr_dash
        self.X_rr_dash = mmg_params.X_rr_dash
        self.X_vvvv_dash = mmg_params.X_vvvv_dash
        self.Y_v_dash = mmg_params.Y_v_dash
        self.Y_r_dash = mmg_params.Y_r_dash
        self.Y_vvv_dash = mmg_params.Y_vvv_dash
        self.Y_vvr_dash = mmg_params.Y_vvr_dash
        self.Y_vrr_dash = mmg_params.Y_vrr_dash
        self.Y_rrr_dash = mmg_params.Y_rrr_dash
        self.N_v_dash = mmg_params.N_v_dash
        self.N_r_dash = mmg_params.N_r_dash
        self.N_vvv_dash = mmg_params.N_vvv_dash
        self.N_vvr_dash = mmg_params.N_vvr_dash
        self.N_vrr_dash = mmg_params.N_vrr_dash
        self.N_rrr_dash = mmg_params.N_rrr_dash

    def X_H(self,u,v,r):
        U=np.sqrt(u**2+v**2)
        v_dash=v/U
        r_dash=r*self.L_pp/U
        return 0.5*self.ρ*self.L_pp*self.d*U**2*self.X_H_dash(v_dash,r_dash)

    def Y_H(self,u,v,r):
        U=np.sqrt(u**2+v**2)
        v_dash=v/U
        r_dash=r*self.L_pp/U
        return 0.5*self.ρ*self.L_pp*self.d*U**2*self.Y_H_dash(v_dash,r_dash)

    def N_H(self,u,v,r):
        U=np.sqrt(u**2+v**2)
        v_dash=v/U
        r_dash=r*self.L_pp/U
        return 0.5*self.ρ*self.L_pp**2*self.d*U**2*self.N_H_dash(v_dash,r_dash)

    def X_H_dash(self,β,r_dash):
        return -self.R_0_dash+self.X_vv_dash*β**2+self.X_vr_dash*β*r_dash+self.X_rr_dash*r_dash**2+self.X_vvvv_dash*β**4

    def Y_H_dash(self,β,r_dash):
        return self.Y_v_dash*β+self.Y_r_dash*r_dash+self.Y_vvr_dash*β**2*r_dash+self.Y_vrr_dash*β*r_dash**2+self.Y_vvv_dash*β**3+self.Y_rrr_dash*r_dash**3

    def N_H_dash(self,β,r_dash):
        return self.N_v_dash*β+self.N_r_dash*r_dash+self.N_vvr_dash*β**2*r_dash+self.N_vrr_dash*β*r_dash**2+self.N_vvv_dash*β**3+self.N_rrr_dash*r_dash**3

    def X_R(self,u,v,r,δ,n_p):
        return -(1-self.t_R)*self.F_N(u,v,r,δ,n_p)*np.sin(δ)

    def Y_R(self,u,v,r,δ,n_p):
        return -(1+self.a_H)*self.F_N(u,v,r,δ,n_p)*np.cos(δ)

    def N_R(self,u,v,r,δ,n_p):
        return -(self.x_R+self.a_H*self.x_H)*self.F_N(u,v,r,δ,n_p)*np.cos(δ)

    def F_N(self,u,v,r,δ,n_p):
        u_p=(1-self.w_P0)*u
        u_inf=u_p*np.sqrt(1+(8*self.K_T(u,n_p))/(np.pi*self.J(u,n_p)**2))
        k_x=self.κ*self.ϵ
        delta_u=u_inf-u_p
        u_RP=u_p+k_x*delta_u
        u_R=np.sqrt(self.η*u_RP**2+(1-self.η)*u_p**2)
        U=np.sqrt(u**2+v**2)
        β=np.arctan(-v/U)
        r_dash=r*self.L_pp/U
        v_R=U*self.γ_R_plus*(β-self.l_r_dash*r_dash)
        U_R=np.sqrt(u_R**2+v_R**2)
        α_R=δ-np.arctan(v_R/u_R)
        return 0.5*self.ρ*self.A_R*U_R**2*self.f_α*np.sin(α_R)

    def X_P(self,u,v,δ,n_p):
        return (1-self.t_P)*self.T_P(u,n_p)

    def K_T(self,u,n_p):
        return self.k_0+self.k_1*self.J(u,n_p)+self.k_2*self.J(u,n_p)**2

    def J(self,u,n_p):
        return u*(1-self.w_P0)/(n_p*self.D_p)

    def T_P(self,u,n_p):
        return self.K_T(u,n_p)*self.ρ*n_p**2*self.D_p**4