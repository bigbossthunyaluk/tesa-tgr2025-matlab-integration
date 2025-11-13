clc
clear all

waypoints = [ 0 0 0 0;
    0 0 -10 1;
    10 10 -10 1;
    10 30 -20 1;
    30 40 -30 1];

% waypoints = [
%     0   0    0     0;
%     5   0   -8     0;
%     12  8   -12    1;
%     20  18  -18    1;
%     32  30  -25    1
% ];
% 
% waypoints = [
%     0   0    0     0;
%     -5  8   -10    1;
%     0   18  -12    1;
%     12  25  -18    1;
%     25  35  -25    1
% ];
% 
% waypoints = [
%     0    0    0      0;
%     8   -6   -6      1;
%     16   6   -12     1;
%     24  -4   -18     1;
%     32   8   -25     1
% ];
% 
% waypoints = [
%     0   0    0     0;
%     10  0   -6     0;
%     20  10  -10    1;
%     25  25  -18    1;
%     25  35  -30    1
% ];
% 
% waypoints = [
%     0    0     0     0;
%     12   5    -10    1;
%     25  18    -20    1;
%     40  32    -28    1;
%     55  50    -35    1
% ];

timepoints = [0 5 10 15 20];

SimulationTime = 20;
timestep = 0.01;

trajectory_coef = min_snap_Coef(waypoints,timepoints);

X =[];
Y = [];
Z = [];
Psi = [];
dX = [];
dY = [];
dZ = [];
dPsi = [];
ddX = [];
ddY = [];
ddZ = [];
ddPsi = [];
dddX = [];
dddY = [];
dddZ = [];
ddddX = [];
ddddY = [];
ddddZ = [];



tX = 0;
tY = 0;
tZ = 0;
tPsi = 0;
tdX = 0;
tdY = 0;
tdZ = 0;
tdPsi = 0;
tddX = 0;
tddY = 0;
tddZ = 0;
tddPsi = 0;
for t = 0:timestep:SimulationTime
[tX,tY,tZ,tPsi,tdX,tdY,tdZ,tdPsi,tddX,tddY,tddZ,tddPsi,tdddX,tdddY,tdddZ,tddddX,tddddY,tddddZ] = getTrajectory(t,trajectory_coef,timepoints);
X = [X tX];
Y = [Y tY];
Z = [Z tZ];
Psi = [Psi tPsi];
dX = [dX tdX];
dY = [dY tdY];
dZ = [dZ tdZ];
dPsi = [dPsi tdPsi];
ddX = [ddX tddX];
ddY = [ddY tddY];
ddZ = [ddZ tddZ];
ddPsi = [ddPsi tddPsi];
dddX = [dddX tdddX];
dddY = [dddY tdddY];
dddZ = [dddZ tdddZ];
ddddX = [ddddX tddddX];
ddddY = [ddddY tddddY];
ddddZ = [ddddZ tddddZ];

end
% f1 = figure;
% plot3(X,Y,Z);
% f1.CurrentAxes.ZDir = 'Reverse';
% xlabel('x')
% ylabel('y')
% zlabel('-z')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Create Variable in Simulick Workspace %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% properties of drones
m = 4.34; 
g = 9.81;
dIdt = zeros(3,3);
I = eye(3,3);
I(1,1) = 0.0820;
I(2,2) = 0.0845;
I(3,3) = 0.1377;
L = 0.315; %length of arms


cf = 8.004e-4;

% K matrix to convert motor speed to force and torques u = K . omega2
K = [1 1 1 1;
    0 -L 0 L;
    L 0 -L 0;
    -cf cf -cf cf];
Kp = eye(3,3);
Kp = 1*Kp;
Kv = eye(3,3);
Kv = 1*Kv;
Kr = eye(3,3);
Kr = 1*Kr;
Kw = eye(3,3);
Kw = 1*Kw;


time = 0:timestep:SimulationTime;
inputStructure.time = time;
inputStructure.signals(1).values = X';
inputStructure.signals(1).dimensions = 1;
inputStructure.signals(2).values = Y';
inputStructure.signals(2).dimensions = 1;
inputStructure.signals(3).values = Z';
inputStructure.signals(3).dimensions = 1;

inputStructure.signals(4).values = dX';
inputStructure.signals(4).dimensions = 1;
inputStructure.signals(5).values = dY';
inputStructure.signals(5).dimensions = 1;
inputStructure.signals(6).values = dZ';
inputStructure.signals(6).dimensions = 1;

inputStructure.signals(7).values = ddX';
inputStructure.signals(7).dimensions = 1;
inputStructure.signals(8).values = ddY';
inputStructure.signals(8).dimensions = 1;
inputStructure.signals(9).values = ddZ';
inputStructure.signals(9).dimensions = 1;

inputStructure.signals(10).values = dddX';
inputStructure.signals(10).dimensions = 1;
inputStructure.signals(11).values = dddY';
inputStructure.signals(11).dimensions = 1;
inputStructure.signals(12).values = dddZ';
inputStructure.signals(12).dimensions = 1;

inputStructure.signals(13).values = ddddX';
inputStructure.signals(13).dimensions = 1;
inputStructure.signals(14).values = ddddY';
inputStructure.signals(14).dimensions = 1;
inputStructure.signals(15).values = ddddZ';
inputStructure.signals(15).dimensions = 1;


inputStructure.signals(16).values = Psi';
inputStructure.signals(16).dimensions = 1;
inputStructure.signals(17).values = dPsi';
inputStructure.signals(17).dimensions = 1;
inputStructure.signals(18).values = ddPsi';
inputStructure.signals(18).dimensions = 1;



out = sim("TESAOffense13.slx");

Xe = out.yout{1}.Values.Data(:,1)';
Ye = out.yout{1}.Values.Data(:,2)';
Ze = out.yout{1}.Values.Data(:,3)';
f2 = figure;
plot3(X,Y,Z,Xe,Ye,Ze);
f2.CurrentAxes.ZDir = 'Reverse';
xlabel('x')
ylabel('y')
zlabel('-z')

% ======= Accuracy mesurement =======

T_vec = [X; Y; Z];
r_vec = [Xe; Ye; Ze];

norm_error = vecnorm(r_vec - T_vec);

norm_T = vecnorm(T_vec);

norm_T = norm_T + eps;

Accuracy_t = 1 - (norm_error ./ (2 * norm_T));

Mean_Accuracy = mean(Accuracy_t);

payload = struct();
payload.droneId = 101;
payload.realLat = NaN;
payload.realLong = NaN;
payload.realAlt = NaN;
payload.desireLat = NaN;
payload.desireLong = NaN;
payload.desireAlt = NaN;
payload.speed = NaN;
payload.timeRange = NaN;
payload.currentTime = NaN;
payload.meanAccuracy = Mean_Accuracy;
options = weboptions('MediaType', 'application/json');
% webwrite("http://192.168.10.181/api/v1/attacks-accuracy", payload, options);
webwrite('http://localhost:3000/drone-location', payload, options);

fprintf('========================================\n');
fprintf('  Accuracy (Mean): %.4f\n', Mean_Accuracy);
fprintf('========================================\n');

% rotor1 = out.yout{2}.Values.Data;
% rotor2 = out.yout{3}.Values.Data;
% rotor3 = out.yout{4}.Values.Data;
% rotor4 = out.yout{5}.Values.Data;
% 
% figure
% plot(time,rotor1);
% figure
% plot(time,rotor2);
% figure
% plot(time,rotor3);
% figure
% plot(time,rotor4);