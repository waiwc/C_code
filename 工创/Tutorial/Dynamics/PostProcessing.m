% ***************************************************************************************
%  File: PostProcessing.m
% 
%  Author:Panpan Luo
% 
%  Description: Post-Processing for VehicleDynamics
% 
%***************************************************************************************

clc,clear;

%% read data from SimOneIOAPISample
if exist ("results.csv")
    res=readtable("results.csv");
end

%% frequency domain analysis bode diagram analysis
if ~isempty(res.SteeringInput)
    maxSteeringAngle=double(540);
    sampling=double(0.0167);
    figure('name','Bode diagram of steer system');
    identificationData=iddata(res.AngularVelZ*(-1),res.SteeringInput/maxSteeringAngle,sampling);
    numberOfZeros=single(1);
    numberOfPoles=single(2);
    steerSys=tfest(identificationData,numberOfPoles,numberOfZeros);
    bode(steerSys);
    grid on
%% compute gain/phase margin and corresponding frequency
    [Gm,Pm,Wgm,Wpm] = margin(steerSys);
    Gm_dB = 20*log10(Gm);
end

%% plot all data from SimOne DynamicsTest
figure('name','SimOne Results Plot');
subplot(3, 6, 1),plot(res.Time_s-res.Time_s(1),res.BrakeInput);
xlabel('BrakeInput');

subplot(3, 6, 2),plot(res.Time_s-res.Time_s(1),res.SteeringInput);
xlabel('SteeringInput');

subplot(3, 6, 3),plot(res.Time_s-res.Time_s(1),res.ThrottleInput');
xlabel('ThrottleInput');

subplot(3, 6, 4),plot(res.Pos_X,res.Pos_Y');
xlabel('Pos');

subplot(3, 6, 5),plot(res.Time_s-res.Time_s(1),res.ForwardSpeed_km_h');
xlabel('ForwardSpeed_km_h');

subplot(3, 6, 6),plot(res.Time_s-res.Time_s(1),res.EngineRotationSpeed');
xlabel('EngineRotationSpeed');

subplot(3, 6, 7),plot(res.Time_s-res.Time_s(1),res.AccX');
xlabel('AccX');

subplot(3, 6, 8),plot(res.Time_s-res.Time_s(1),res.AccY');
xlabel('AccY');

subplot(3, 6, 9),plot(res.Time_s-res.Time_s(1),res.AngularVelX');
xlabel('AngularVelX');

subplot(3, 6, 10),plot(res.Time_s-res.Time_s(1),res.AngularVelY');
xlabel('AngularVelY');

subplot(3, 6, 11),plot(res.Time_s-res.Time_s(1),res.AngularVelZ');
xlabel('AngularVelZ');

subplot(3, 6, 12),plot(res.Time_s-res.Time_s(1),res.wheelSpeedFL');
xlabel('wheelSpeedFL');

subplot(3, 6, 13),plot(res.Time_s-res.Time_s(1),res.wheelSpeedFR');
xlabel('wheelSpeedFR');

subplot(3, 6, 14),plot(res.Time_s-res.Time_s(1),res.wheelSpeedRL');
xlabel('wheelSpeedRL');

subplot(3, 6, 15),plot(res.Time_s-res.Time_s(1),res.wheelSpeedRR');
xlabel('wheelSpeedRR');

subplot(3, 6, 16),plot(res.Time_s-res.Time_s(1),res.Ax_m_s2');
xlabel('Ax_m_s2');

subplot(3, 6, 17),plot(res.Time_s-res.Time_s(1),res.Ay_m_s2');
xlabel('Ay_m_s2');

disp('post-processing finished');

%% plot all data from SimOne in seperate figures
% figure('name','BrakeInput');
% plot(res.Time_s,res.BrakeInput);
% 
% figure('name','SteeringInput');
% plot(res.Time_s,res.SteeringInput);
% 
% figure('name','ThrottleInput');
% plot(res.Time_s,res.ThrottleInput');
% 
% figure('name','Pos');
% plot(res.Pos_X,res.Pos_Y');
% 
% figure('name','ForwardSpd');
% plot(res.Time_s,res.ForwardSpeed_km_h');
% 
% figure('name','EngineSpeedRPM');
% plot(res.Time_s,res.EngineRotationSpeed');
% 
% figure('name','AccX');
% plot(res.Time_s,res.AccX');
% 
% figure('name','AccY');
% plot(res.Time_s,res.AccY');
% 
% figure('name','AngularVelX');
% plot(res.Time_s,res.AngularVelX');
% 
% figure('name','AngularVelY');
% plot(res.Time_s,res.AngularVelY');
% 
% figure('name','AngularVelZ');
% plot(res.Time_s,res.AngularVelZ');
% 
% figure('name','wheelSpeedFL');
% plot(res.Time_s,res.wheelSpeedFL');
% 
% figure('name','wheelSpeedFL');
% plot(res.Time_s,res.wheelSpeedFR');
% 
% figure('name','wheelSpeedRR');
% plot(res.Time_s,res.wheelSpeedRR');
% 
% figure('name','wheelSpeedRL');
% plot(res.Time_s,res.wheelSpeedRL');