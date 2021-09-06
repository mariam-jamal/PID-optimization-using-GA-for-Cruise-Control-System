function J = pidtest(sys,dt,PID_const,plotflag)
%rng default % For reproducibility
s = tf('s');
K_GA = PID_const(1) + PID_const(2)/s + PID_const(3)*s/(1+.001*s);
Loop = series(K_GA,sys);
ClosedLoop = feedback(Loop,1);
t = 0:dt:20;
[y,t] = step(ClosedLoop,t);
e = 1 - step(ClosedLoop,t);

R = 1 - 0.3032*PID_const(1) + 0.2813*PID_const(2) - 0.0229*PID_const(3);
I = 0.2435*PID_const(1) + 0.0742*PID_const(2) - 0.5252*PID_const(3);
J = abs(R) + abs(I);

if plotflag == 1
    step(ClosedLoop,t)
    stepinfo(ClosedLoop)
    h = findobj(gcf,'type','line');
    set(h,'linewidth',2);
    drawnow
end