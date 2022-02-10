clear all;
close all;
addpath('../../../auxiliary_funs/');


%% Params
n_hp_per_split = 500;
n_split = 10;
dataset = './../data/data_learning.mat';
saveloc = './../data/approx_max_RPI_sim_based.mat';


%% Load data
load(dataset);
X = X';
nx = size(X,2);
ns = size(X,1);


%% Run computation in loop for shorter computation times
rng(13209); % fix seed

% initialize final hyperplanes
RPI_A = zeros(0, nx);
RPI_b = zeros(0, 1);
for i = 1:n_split
    
    disp(strcat('Compute split', num2str(i), '/', num2str(n_split)));
    
    % directions of hyperplanes
    RPI_A_cur = rand(n_hp_per_split, nx) * 2 - 1;

    % Compute tight bounds
    RPI_b_cur = polyhedral_approx(X, RPI_A_cur);
    
    % Append solution
    RPI_A = [RPI_A; RPI_A_cur];
    RPI_b = [RPI_b; RPI_b_cur];
    
end


%% save data
save(saveloc, 'RPI_A', 'RPI_b');