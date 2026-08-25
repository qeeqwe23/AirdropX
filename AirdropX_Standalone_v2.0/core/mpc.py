from __future__ import annotations
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import minimize
from .model_bank import Vertex

@dataclass
class MPCSolution:
    u: np.ndarray
    du: np.ndarray
    U: np.ndarray
    feasible: bool
    solve_time_s: float
    predicted: np.ndarray

class DenseBoxMPC:
    """Numerical port of airdropx_phys_mpc_condense/solve (Np=Nc=100).

    The QP is expressed in input deviations from the scheduled physical trim.
    An explicit warm start is accepted so the Paper cfg/recovery logic can use
    exactly one receding-horizon warm sequence across transient cost banks.
    """
    def __init__(self,v:Vertex,horizon:int=100):
        self.A=np.asarray(v.A,float); self.B=np.asarray(v.B,float); self.Q=np.asarray(v.Q,float); self.R=np.asarray(v.R,float); self.P=np.asarray(v.P,float)
        self.xr=np.asarray(v.xref,float).reshape(7); self.ur=np.asarray(v.uref,float).reshape(2); self.ss=np.asarray(v.state_scale,float).reshape(7)
        self.N=int(horizon); self.n=7; self.m=2
        N=self.N; n=self.n; m=self.m; A=self.A; B=self.B
        Phi=np.zeros((n*N,n)); Gamma=np.zeros((n*N,m*N))
        powers=[np.eye(n)]
        for _ in range(1,N+1): powers.append(A@powers[-1])
        for i in range(1,N+1):
            rows=slice((i-1)*n,i*n); Phi[rows]=powers[i]
            for j in range(1,i+1): Gamma[rows,(j-1)*m:j*m]=powers[i-j]@B
        Qbar=np.kron(np.eye(N),self.Q); Qbar[-n:,-n:]=self.P; Rbar=np.kron(np.eye(N),self.R)
        H=2*(Gamma.T@Qbar@Gamma+Rbar); self.H=.5*(H+H.T); self.Fx=2*(Gamma.T@Qbar@Phi)
        Gdist=np.zeros((n*N,n*N))
        for i in range(1,N+1):
            rows=slice((i-1)*n,i*n)
            for j in range(1,i+1): Gdist[rows,(j-1)*n:j*n]=powers[i-j]
        self.Phi=Phi; self.Gamma=Gamma; self.Qbar=Qbar; self.Gdist=Gdist; self.Fdist=2*(Gamma.T@Qbar@Gdist)
        umin=np.array([-1.,0.]); umax=np.array([1.,1.]); self.lb=np.tile(umin-self.ur,N); self.ub=np.tile(umax-self.ur,N)
        self.warm=np.zeros(m*N)

    def reweighted(self,q_multiplier,r_multiplier):
        """Port of airdropx_phys_mpc_reweight_v132 without duplicating dynamics.

        A/B, trim, terminal P, hard bounds, horizon and disturbance propagation
        stay identical; only finite-horizon stage Q/R change. The final Qbar
        block remains the certified base terminal P.
        """
        qm=np.asarray(q_multiplier,float).reshape(self.n); rm=np.asarray(r_multiplier,float).reshape(self.m)
        if np.any(qm<=0) or np.any(rm<=0): raise ValueError('recovery multipliers must be positive')
        out=object.__new__(DenseBoxMPC)
        for name in ('A','B','P','xr','ur','ss','N','n','m','Phi','Gamma','Gdist','lb','ub'):
            setattr(out,name,getattr(self,name))
        Dq=np.diag(np.sqrt(qm)); Dr=np.diag(np.sqrt(rm)); out.Q=Dq@self.Q@Dq; out.R=Dr@self.R@Dr
        out.Qbar=self.Qbar.copy()
        for i in range(out.N-1):
            rr=slice(i*out.n,(i+1)*out.n); out.Qbar[rr,rr]=out.Q
        out.Qbar[-out.n:,-out.n:]=self.P
        Rbar=np.kron(np.eye(out.N),out.R)
        H=2*(out.Gamma.T@out.Qbar@out.Gamma+Rbar); out.H=.5*(H+H.T)
        out.Fx=2*(out.Gamma.T@out.Qbar@out.Phi); out.Fdist=2*(out.Gamma.T@out.Qbar@out.Gdist)
        out.warm=np.zeros(out.m*out.N)
        return out

    @staticmethod
    def state_error(x,xr):
        d=np.asarray(x,float).reshape(7)-np.asarray(xr,float).reshape(7)
        for i in (2,3): d[i]=np.arctan2(np.sin(d[i]),np.cos(d[i]))
        return d

    @staticmethod
    def shift_warm(U,m=2):
        z=np.asarray(U,float).reshape(-1)
        return np.r_[z[m:],z[-m:]]

    @staticmethod
    def rebase_warm(warm_old,old_ctrl,new_ctrl):
        z=np.asarray(warm_old,float).reshape(-1)
        if z.size!=old_ctrl.m*old_ctrl.N or old_ctrl.m!=new_ctrl.m or old_ctrl.N!=new_ctrl.N or not np.isfinite(z).all():
            return np.zeros(new_ctrl.m*new_ctrl.N)
        old_abs=z+np.tile(old_ctrl.ur,old_ctrl.N)
        return np.clip(old_abs-np.tile(new_ctrl.ur,new_ctrl.N),new_ctrl.lb,new_ctrl.ub)

    def solve(self,x,g_seq=None,warm=None,xref_override=None):
        xr=self.xr if xref_override is None else np.asarray(xref_override,float).reshape(7)
        dx=self.state_error(x,xr); f=self.Fx@dx
        base=self.Phi@dx
        if g_seq is not None:
            gvec=np.asarray(g_seq,float).reshape(7,self.N,order='F').reshape(-1,order='F')
            f += self.Fdist@gvec
            base += self.Gdist@gvec
        w=self.warm if warm is None else np.asarray(warm,float).reshape(-1)
        if w.size!=self.m*self.N or not np.isfinite(w).all(): raise ValueError('bad MPC warm start')
        x0=np.clip(w,self.lb,self.ub)
        t0=time.perf_counter()
        fun=lambda U: .5*float(U@self.H@U)+float(f@U)
        jac=lambda U: self.H@U+f
        r=minimize(fun,x0,jac=jac,bounds=list(zip(self.lb,self.ub)),method='L-BFGS-B',options={'maxiter':80,'ftol':1e-11,'gtol':1e-8,'maxls':30})
        U=np.clip(np.asarray(r.x,float),self.lb,self.ub); elapsed=time.perf_counter()-t0
        feasible=bool(r.success and np.isfinite(U).all() and np.max(self.lb-U)<=1e-7 and np.max(U-self.ub)<=1e-7)
        du=U[:2]; u=np.clip(self.ur+du,[-1.,0.],[1.,1.]); pred=(base+self.Gamma@U).reshape(self.N,7).T+xr[:,None]
        self.warm=self.shift_warm(U,self.m)
        return MPCSolution(u,du,U,feasible,elapsed,pred)
