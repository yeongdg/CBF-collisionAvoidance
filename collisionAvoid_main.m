% doubleIntegeral2D_main.m
close all;
clear all;
clc;



% System Parameter Limits
params.obs.x = 1;
params.obs.y = 1;
params.obs.Radius = 0.25;

params.target.x = 2;
params.target.y = 2.5;
params.K = 10;



% Define the initial conditions
x0 = [0; 0];        % [position; velocity] 



% Simulation Parameters
dt = 0.01;
tEnd = 5;
runTime = [0 tEnd];



% Plot the results
figure;
hold on

viscircles([params.obs.x params.obs.y], params.obs.Radius);
text(params.obs.x,params.obs.y,'X','Color','r');
text(params.obs.x,params.obs.y-0.1,'X_o_b_s');

text(x0(1),x0(2),'X','Color','b');
text(x0(1)-0.2,x0(2),'X(0)');
text(params.target.x,params.target.y,'X','Color','b');
text(params.target.x+0.1,params.target.y,'X_t_g_t');


params.gamma = 1;       % CBF extended class K function gamma
% Instantiate the DoubleIntegrator system and CBFController
controller = CBFcollisionAvoid(params);
% Define the ODE function for integration
odeFunc = @(t, x) controller.dynamics(x, t);
% Run the simulation using ode45
[t, x] = ode45(odeFunc, runTime, x0);
% plot 
plot(x(:,1),x(:,2));

params.gamma = 10;       % CBF extended class K function gamma
% Instantiate the DoubleIntegrator system and CBFController
controller = CBFcollisionAvoid(params);
% Define the ODE function for integration
odeFunc = @(t, x) controller.dynamics(x, t);
% Run the simulation using ode45
[t, x] = ode45(odeFunc, runTime, x0);
% plot 
plot(x(:,1),x(:,2));

params.gamma = 1000;       % CBF extended class K function gamma
% Instantiate the DoubleIntegrator system and CBFController
controller = CBFcollisionAvoid(params);
% Define the ODE function for integration
odeFunc = @(t, x) controller.dynamics(x, t);
% Run the simulation using ode45
[t, x] = ode45(odeFunc, runTime, x0);
% plot 
plot(x(:,1),x(:,2));


legend('\gamma=1','\gamma=10','\gamma=1000')


grid on
xlim([(x0(1)-0.5) (params.target.y+0.25)]);
ylim([(x0(2)-0.25) (params.target.y+0.25)]);

xlabel('Position x (m)');
ylabel('Position y (m)');

pic = gcf;
exportgraphics(pic,'img/obstacleAvoidance.jpg','Resolution',600);