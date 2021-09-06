clear; close; clc;
%% System Modelling
% Transfer function
Gain = 2.4767;
Zero = [];
Pole = [-0.0476 -1 -5];
CC_sys = zpk(Zero,Pole,Gain);
fprintf("Performance charactristics of Open loop system:")
figure(1)
step(CC_sys)
box on, grid on
set(gcf,'Position',[100 100 550 250])
set(gcf,'PaperPositionMode','auto')
print('Fig 1', '-dpng');
CC_sys_char = stepinfo(CC_sys)

%% Tuning PID using Ziegler-Nichols Method

%--Calculating total gain and different gain parameters according to type of controller--%
% Type = 1 for P controller
% Type = 2 for PI controller
% Type = 3 for PID controller
[Wcp,Kpp,Tip,Tdp]=ZN(CC_sys,1);
[Wcpi,Kppi,Tipi,Tdpi]=ZN(CC_sys,2);
[Wcpid,Kppid,Tipid,Tdpid]=ZN(CC_sys,3);

%Closed Loop transfer function
ZN_P_controller=feedback(series(CC_sys,Wcp),1);
ZN_PI_controller=feedback(series(CC_sys,Wcpi),1);
ZN_PID_controller=feedback(series(CC_sys,Wcpid),1);

%Gain parameters calculated from ZN method
ZN_P_gains = [Kpp Kpp/Tip Kpp*Tdp];
ZN_PI_gains = [Kppi Kppi/Tipi Kppi*Tdpi];
ZN_PID_gains = [Kppid Kppid/Tipid Kppid*Tdpid];

%Plotting
figure(2)
t=0:0.01:50;
step(ZN_P_controller,ZN_PI_controller,ZN_PID_controller,t),
legend('P','PI','PID');
box on, grid on
set(gcf,'Position',[100 100 550 250])
set(gcf,'PaperPositionMode','auto')
print('Fig 2', '-dpng');
fprintf("Performance charactristics of Z-N tuned PID Controller:")
ZN_sys_char = stepinfo(ZN_PID_controller)


%% Tuning PID using Genetic Algorithm
s = tf('s');
dt = 0.001;
popsize = 40;
MaxGenerations = 25;
rng(1,'twister') % for reproducibility

% population = rand(popsize,3);
load randpop.mat

options = optimoptions(@ga,'PopulationSize',popsize,'MaxGenerations',MaxGenerations,'InitialPopulation',population,'OutputFcn',@myfun);
[x,fval,exitflag,output,population,scores] = ga(@(K_GA)pidtest(CC_sys,dt,K_GA,0),3,-eye(3),zeros(3,1),[],[],[3 0.1 3],[4 0.25 4],[],options);

%options = optimoptions(@ga,'PopulationSize',popsize,'MaxGenerations',MaxGenerations,'OutputFcn',@myfun,'Vectorized','off');
%[x,fval] = ga(@(K_GA)pidtest(CC_sys,dt,K_GA,0),3,-eye(3),zeros(3,1),[],[],[3 0.1 3],[4 0.25 4],[],options);
disp(x)
PID_cont=x(1) + x(2)/s + x(3)*s/(1+.001*s);
GA_PID = feedback(PID_cont*CC_sys,1);

%Comparison of ZN tuned PID and GA tuned PID
figure(3)
step(ZN_PID_controller,GA_PID)
legend('ZN optimised PID','GA optimised PID','Location','southeast')
box on, grid on
set(gcf,'Position',[100 100 550 250])
set(gcf,'PaperPositionMode','auto')
print('Fig 3', '-dpng');
fprintf("Performance charactristics of GA tuned PID Controller:")
GA_sys_char = stepinfo(GA_PID)

%% Cost function across Generations as GA optimises PID gains

load history.mat
for k=1:MaxGenerations
    sortedcost(:,k) = sort(cost(:,k));
end
figure(4)
imagesc(log(sortedcost(:,1:MaxGenerations)))
xlabel('Generation')
ylabel('Sorted Individual')
colorbar
set(gcf,'Position',[100 100 600 300])
set(gcf,'PaperPositionMode','auto')
print('Fig 4', '-dpng');

%% PID gains from GA
figure(5)
    hold on
    for k=1:MaxGenerations
        for j=1:popsize
            scatter3(history(j,1,k),history(j,2,k),history(j,3,k),15,[(MaxGenerations-k)/MaxGenerations 0.25 k/MaxGenerations],'filled');
        end
    end
      [B,I] = sort(cost(:,MaxGenerations));  
      scatter3(history(I(1),1,MaxGenerations),history(I(1),2,MaxGenerations),history(I(1),3,MaxGenerations),100,[0 0 0],'filled')
        view(69,24)
    box on
    xlabel('Kp')
    ylabel('Ki')
    zlabel('Kd')
set(gcf,'Position',[100 100 350 250])
set(gcf,'PaperPositionMode','auto')
print('Fig 5', '-dpng');

%% Plot Generation 1
gen = 1;
t = 0:dt:10;
s = tf('s');
figure(6)
hold on
for k=1:popsize
    K = history(k,1,gen) + history(k,2,gen)/s + history(k,3,gen)*s/(1+.001*s);
    L = series(K,CC_sys);
    CL = feedback(L,1);
    [y,t] = step(CL,t);
    plot(t,y,'LineWidth',1.2);
end
xlabel('Time (seconds)')
ylabel('Amplitude')
box on, grid on
set(gcf,'Position',[100 100 550 250])
set(gcf,'PaperPositionMode','auto')
print('Fig 6', '-dpng');

%% Plot Generation 25
gen = 25;
t = 0:dt:10;
s = tf('s');
figure(7)
hold on
for k=1:popsize
    K = history(k,1,gen) + history(k,2,gen)/s + history(k,3,gen)*s/(1+.001*s);
    L = series(K,CC_sys);
    CL = feedback(L,1);
    [y,t] = step(CL,t);
    plot(t,y,'LineWidth',1.2);
end
xlabel('Time (seconds)')
ylabel('Amplitude')
box on, grid on
set(gcf,'Position',[100 100 550 250])
set(gcf,'PaperPositionMode','auto')
print('Fig 7', '-dpng');

%{
%% Plot BEST of each Generation
figure
t = 0:dt:20;
s = tf('s');
for gen=1:MaxGenerations
    [B,I] = sort(cost(:,gen));
    K = history(I(1),1,gen) + history(I(1),2,gen)/s + history(I(1),3,gen)*s/(1+.001*s);
    L = series(K,CC_sys);
    CL = feedback(L,1);
    [y,t] = step(CL,t);
    subplot(3,1,1), hold on
    plot(t,y,'LineWidth',1+.1*gen,'Color',[(MaxGenerations-gen)/MaxGenerations 0 gen/MaxGenerations],'LineWidth',1.2);
    xlabel('Time (seconds)')
    ylabel('y')
    box on, grid on
    subplot(3,1,2), hold on
    CTRLtf = K/(1+K*CC_sys);
    u = lsim(K,1-y,t);
    plot(t,u,'LineWidth',1+.1*gen,'Color',[(MaxGenerations-gen)/MaxGenerations 0 gen/MaxGenerations],'LineWidth',1.2);
    xlabel('Time (seconds)')
    ylabel('u')
    ylim([-1 2])
    box on, grid on
    subplot(3,1,3), hold on
    R = 1 - 0.3032*history(I(1),1,gen) + 0.2813*history(I(1),2,gen) - 0.0229*history(I(1),3,gen);
    I = 0.2435*history(I(1),1,gen) + 0.0742*history(I(1),2,gen) - 0.5252*history(I(1),3,gen);
    J = abs(R) + abs(I);
    plot(t,J,'LineWidth',1+.1*gen,'Color',[(MaxGenerations-gen)/MaxGenerations 0 gen/MaxGenerations],'LineWidth',1.2);
    xlabel('Time (seconds)')
    ylabel('J')
end
box on, grid on
set(gcf,'Position',[100 100 550 350])
set(gcf,'PaperPositionMode','auto')
%print('Cost_func5', '-dpng');
%}
