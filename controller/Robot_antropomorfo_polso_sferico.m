clear all
close all
clc

%% MANIPOLATORE ANTROPOMORFO CON POLSO SFERICO

teta = 0; %Variabile di giunto
ap=[0 210 30 0 5.5 0];   %mm
alphap=[-pi/2 0 pi/2 -pi/2 pi/2 0]; %mm
dp=[80 0 30 221.5 0 23.7];  %mm

for i=1:1:6
B(i)=Link([teta dp(i) ap(i) alphap(i)]);
end

%tetap=[pi/6 pi/3 pi/2 pi/3 0 pi/6];
q0 = [0 0 0 0 0 0];
qf=[0 pi/4 -pi/2 -pi/6 pi/2 0];

MyRobot=SerialLink(B,'name','Manipolatore antropomorfo con polso');
% T_man_antrop_polso=Man_antrop_polso.fkine(qf);
figure(1)
MyRobot.plot(q0);
figure(2)
MyRobot.teach('eul');

t = 0:0.0001:0.001;
[q,qd,qdd] = mtraj(@lspb,q0,qf,t);
figure(3)
plot(MyRobot,q);
figure(4)

plot(t',qd(:,4))
plot(t',qd(:,4))

%% Parametri cinematici e dinamici
