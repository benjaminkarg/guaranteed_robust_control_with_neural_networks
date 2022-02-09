clear all;
close all;
addpath('../../../auxiliary_funs/');


%% Params
dataset = './../data/data_learning.mat';
saveloc = './../data/approx_max_RPI_sim_based.mat';


%% Load data
load(dataset);
X = X';
nx = size(X,2);
ns = size(X,1);


%% Generate direction of hyperplanes
n_comb = 10;
RPI_A = combinator(n_comb, 2, 'p', 'r');
RPI_A = (RPI_A - 1) / (n_comb - 1) * 2 - 1;
RPI_A = RPI_A(any(RPI_A, 2), :);


%% Compute tight bounds
RPI_b = polyhedral_approx(X, RPI_A);


%% save data
RPI = Polyhedron(RPI_A, RPI_b);
RPI.minHRep;
RPI_A = RPI.A;
RPI_b = RPI.b;
save(saveloc, 'RPI_A', 'RPI_b');


%% plot data and set
plot(RPI);
hold on;
scatter(X(:,1), X(:,2));