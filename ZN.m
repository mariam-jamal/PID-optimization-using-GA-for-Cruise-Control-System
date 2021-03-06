function [Wc,Kp,Ti,Td] = ZN(sys,type)
[Kc,Pm,Wc,Wm]=margin(sys);
pu=2*pi/Wc;
s=tf('s');
if type == 1
   %P-Controller
    Kp=Kc*0.5;
    Ti=100000;
    Td=0; 
    Wc=Kp*(1+(1/(Ti*s))+Td*s);
end

if type == 2
    %PI-Controller
    Kp=Kc*0.45;
    Ti=pu*0.83;
    Td=0;
    Wc=Kp*(1+(1/(Ti*s))+Td*s);
end

if type == 3
    %PID-Controller
    Kp=Kc*0.59;
    Ti=pu*0.5;
    Td=pu*0.12;
    Wc=Kp*(1+(1/(Ti*s))+Td*s);
end




