%% ADRC-OF Demo Script
% This script demonstrates how to use the ADRC-based Optimization Framework
% on several standard benchmark functions.
%
% Paper: A Closed-Loop Active Disturbance Rejection Control-Based
%        Optimization Framework for Constrained Engineering Optimization
% Authors: Jing Xiang, Yingkai Ma, Wentao Zhou, Yuxuan Chen, Hua Guo, Guihua Xia

clear; clc; close all;

fprintf('========================================\n');
fprintf('  ADRC-OF Demo Script\n');
fprintf('========================================\n\n');

%% Settings
D = 10;         % Dimensionality
N = 50;         % Population size
T = 1000;       % Max iterations
lb = -100 * ones(1, D);
ub = 100 * ones(1, D);

%% Test 1: Sphere Function (Unimodal)
fprintf('Test 1: Sphere Function (D=%d)\n', D);
fprintf('  f(x) = sum(x^2)\n');

fun1 = @(x) sum(x.^2, 2);
[X1, F1, curve1] = ADRCOF(fun1, D, lb, ub, N, T);
fprintf('  Best fitness: %.4e\n\n', F1);

%% Test 2: Rastrigin Function (Multimodal, separable)
fprintf('Test 2: Rastrigin Function (D=%d)\n', D);
fprintf('  f(x) = 10*D + sum(x^2 - 10*cos(2*pi*x))\n');

fun2 = @(x) 10*D + sum(x.^2 - 10*cos(2*pi*x), 2);
[X2, F2, curve2] = ADRCOF(fun2, D, lb, ub, N, T);
fprintf('  Best fitness: %.4e\n\n', F2);

%% Test 3: Sphere with Custom Parameters (Tuned)
fprintf('Test 3: Sphere Function with Tuned Parameters (D=%d)\n', D);
fprintf('  kp=0.39, Wo=0.08, b=0.02, r=0.06\n');

[X3, F3, curve3] = ADRCOF(fun1, D, lb, ub, N, T, ...
    'SEF_kp', 0.39, 'ESO_Wo', 0.08, 'gain', 0.02, 'Err_td', 0.06);
fprintf('  Best fitness: %.4e\n\n', F3);

%% Test 4: Verbose Mode Demo (smaller budget for demo)
fprintf('Test 4: Verbose Mode Demo (D=5, N=30, T=100)\n');

fun4 = @(x) sum(x.^2, 2);
[X4, F4, curve4] = ADRCOF(fun4, 5, -10*ones(1,5), 10*ones(1,5), 30, 100, ...
    'Verbose', true);
fprintf('  Best fitness: %.4e\n\n', F4);

%% Plot Convergence Curves
figure('Position', [100 100 900 600]);

subplot(2,3,1);
semilogy(curve1, 'LineWidth', 1.2);
title('Sphere (Default)'); xlabel('Iteration'); ylabel('Fitness');
grid on;

subplot(2,3,2);
semilogy(curve2, 'LineWidth', 1.2);
title('Rastrigin (Default)'); xlabel('Iteration'); ylabel('Fitness');
grid on;

subplot(2,3,3);
semilogy(curve3, 'LineWidth', 1.2);
title('Sphere (Tuned)'); xlabel('Iteration'); ylabel('Fitness');
grid on;

subplot(2,3,[4 6]);
semilogy(curve1, 'LineWidth', 1.2); hold on;
semilogy(curve3, 'LineWidth', 1.2);
semilogy(curve4, 'LineWidth', 1.2);
title('Comparison'); xlabel('Iteration'); ylabel('Fitness');
legend('Sphere Default', 'Sphere Tuned', 'Sphere D=5 Verbose', ...
    'Location', 'northeast');
grid on;

fprintf('========================================\n');
fprintf('  Demo complete. Check the figure window.\n');
fprintf('========================================\n');
