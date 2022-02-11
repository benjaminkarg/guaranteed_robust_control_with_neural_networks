clear all;
close all;
addpath('../../../auxiliary_funs/');


%% Params
n_hp = 4000;
dataset = './../data/min_RPI_sim_data.mat';
saveloc = './../data/approx_min_RPI_sim_based.mat';


%% Load data
load(dataset);
nx = size(X,2);
ns = size(X,1);


%% Generate direction of hyperplanes
n_comb = 3;
RPI_A = combinator(n_comb, nx, 'p', 'r');
RPI_A = (RPI_A - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1


%% Compute tight bounds
RPI_b = polyhedral_approx(X, RPI_A);


%% save data
save(saveloc, 'RPI_A', 'RPI_b');