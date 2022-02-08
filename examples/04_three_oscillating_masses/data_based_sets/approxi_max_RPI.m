clear all;
close all;
addpath('../../../auxiliary_funs/');


%% Params
n_hp = 4000;
dataset = './../data/max_RPI_sim_data.mat';
saveloc = './../data/approx_max_RPI_sim_based.mat';


%% Load data
load(dataset);
nx = size(X,2);
ns = size(X,1);


%% Generate direction of hyperplanes
rng(13209);
RPI_A = rand(n_hp, nx) * 2 - 1;


%% Compute tight bounds
RPI_b = polyhedral_approx(X, RPI_A);


%% save data
save(saveloc, 'RPI_A', 'RPI_b');