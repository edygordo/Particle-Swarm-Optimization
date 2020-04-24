clc;
clear;
close all;

%% Problem definition

problem.CostFunction = @(x) Sphere(x); % can be a general cost function
problem.nVar = 10; %No. of unknown (Decision) variables % in this 5 dimensional space
problem.VarMin = -10; % Lower Bound of Decision Variables
problem.VarMax =  10; % Upper Bound of Decision Variables

%% Parameters of PSO

% Constriction Co-efficients by Clerc and Kennedy, 2002
% k=1,phi1 = 2.05,phi2 = 2.05, x(chi) = formula
% w = x, c1 = x * phi1 , c2 = x * phi2;

kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = 2*kappa/abs(2-phi-sqrt(phi^2 - 4*phi));

params.MaxIt = 1000; % Maximum Number of Iterations
params.nPop = 50; % Population size (Swarm Size)
params.w = chi; % Inertia co-efficient
params.wdamp = 1; % if w=1 then insert wdamp = 0.99 if Constriction co-effiencts introduced then wdamp = 1
params.c1 = chi * phi1; % Personal Acceleration Coefficient
params.c2 = chi * phi2; % Social Acceleration Coefficient
params.ShowIterInfo = false;

%% Calling PSO

out = PSO(problem,params);
BestSol = out.BestSol;
BestCosts = out.BestCosts;

%% Results

figure;

% plot(BestCosts, 'LineWidth', 2);
semilogy(BestCosts);
xlabel('Iteration');
ylabel('Best Cost');
grid on;