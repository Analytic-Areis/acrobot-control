import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import expm
plt.rcParams.update({'font.family':'serif','font.serif':['STIXGeneral'],'mathtext.fontset':'stix','font.size':8,'axes.linewidth':0.5})
g=9.81
def f(t,x,tau=0.0):
    q1,q2,d1,d2=x; c=np.cos(q2); s=np.sin(q2)
    M=np.array([[2.663+2*c,1.33+c],[1.33+c,1.33]])
    C=np.array([[-2*s*d2,-s*d2],[s*d1,0]])
    tg=-g*np.array([1.5*np.sin(q1)+np.sin(q1+q2),np.sin(q1+q2)])
    dd=np.linalg.solve(M,tg+np.array([0,tau])-C@[d1,d2]); return [d1,d2,*dd]
def E(x):
    q1,q2,d1,d2=x; c=np.cos(q2); M=np.array([[2.663+2*c,1.33+c],[1.33+c,1.33]]); dq=np.array([d1,d2])
    return 0.5*dq@M@dq - g*(1.5*np.cos(q1)+np.cos(q1+q2))
def lin(q1,q2):
    c=np.cos(q2); M=np.array([[2.663+2*c,1.33+c],[1.33+c,1.33]]); ph=q1+q2
    K=-g*np.array([[1.5*np.cos(q1)+np.cos(ph),np.cos(ph)],[np.cos(ph),np.cos(ph)]])
    P=np.linalg.solve(M,K); return np.block([[np.zeros((2,2)),np.eye(2)],[P,np.zeros((2,2))]])
out={}
fig,axs=plt.subplots(2,1,figsize=(3.45,2.4))
for ax,(nm,xs,dx,T) in zip(axs,[('E1',np.array([0,0,0,0.]),np.array([0.1,-0.1,0,0]),8.0),('E3',np.array([np.pi,0,0,0.]),np.array([0.001,0,0,0]),1.2)]):
    t=np.linspace(0,T,1201)
    sol=solve_ivp(f,(0,T),xs+dx,t_eval=t,rtol=1e-10,atol=1e-12,method='DOP853')
    A=lin(xs[0],xs[1]); xl=np.array([expm(A*ti)@dx for ti in t]).T
    xn=sol.y-xs[:,None]
    err=np.abs(xn[:2]-xl[:2]).max(axis=1); amp=np.abs(xn[:2]).max(axis=1)
    out[nm]=(err,amp)
    ax.plot(t,np.degrees(xn[0]),color='#1f77b4',lw=1.2,label=r'$\tilde q_1$ nonlinear')
    ax.plot(t,np.degrees(xl[0]),'--',color='k',lw=0.8,label=r'$\tilde q_1$ linear')
    ax.plot(t,np.degrees(xn[1]),color='#e66100',lw=1.2,label=r'$\tilde q_2$ nonlinear')
    ax.plot(t,np.degrees(xl[1]),':',color='k',lw=0.8,label=r'$\tilde q_2$ linear')
    ax.set_ylabel('[deg]'); ax.grid(alpha=0.3,lw=0.4); ax.set_xlim(0,T)
    if nm=='E3':
        ax.set_ylim(-15,15)
        # time until linear/nonlinear differ by 10% of response
        dev=np.abs(xn[0]-xl[0]); idx=np.argmax(np.abs(xn[0])>np.radians(10)); out['E3t10']=t[idx]; out['E3err_at10']=np.degrees(dev[idx])
        out['E3err1deg']=t[np.argmax(np.abs(xn[0])>np.radians(1))]
        e1=np.argmax(np.abs(xn[0])>np.radians(1)); out['E3relerr1deg']=dev[e1]/abs(xn[0][e1])
    ax.set_title(r'(a) Near $E_1$: $\tilde q(0)=(0.1,-0.1)$ rad' if nm=='E1' else r'(b) Near $E_3$: $\tilde q_1(0)=10^{-3}$ rad',fontsize=8)
h,l=axs[0].get_legend_handles_labels(); fig.legend(h,l,fontsize=6.8,ncol=4,loc='upper center',frameon=False,handlelength=1.6,columnspacing=0.7,bbox_to_anchor=(0.53,1.0))
axs[1].set_xlabel('time [s]'); fig.tight_layout(h_pad=0.5,rect=(0,0,1,0.9)); fig.savefig('sim.pdf'); fig.savefig('sim.png',dpi=150)
# energy
x0=np.array([2.0,-1.0,0.5,1.0]); sol=solve_ivp(f,(0,20),x0,rtol=1e-10,atol=1e-12,method='DOP853',dense_output=True)
Es=np.array([E(sol.sol(ti)) for ti in np.linspace(0,20,2001)]); out['Edrift']=np.abs(Es-Es[0]).max()/abs(Es[0]); out['E0']=Es[0]
# relative error at E1
print(out)
