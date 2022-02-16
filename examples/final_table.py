import numpy as np
import scipy.io as sio
import tensorflow as tf
import pdb


""" Params """
locs = ['./02_one_oscillating_mass/', './03_two_oscillating_masses/', './04_three_oscillating_masses/']
names = ['one', 'two', 'three']


""" Network sizes """
networks = {}
for name, loc in zip(names, locs):
    data = tf.keras.models.load_model(loc + 'data/nn_controller.h5')
    networks[loc] =  {'L': int(len(data.weights)/2), 'n_l': data.weights[0].shape[1]}
print(networks)


""" Algorithm verification candidate set """
verification_candidate = {}
for name, loc in zip(names, locs):
    data = sio.loadmat(loc + 'data/verification_MRCI_candidate.mat')
    verification_candidate[name] = {'r': data['H'].shape[1] - 1, 'comp_time': float(data['comp_time']/60)}
print(verification_candidate)


""" Algorithm approximate min RPI """
min_RPI = {}
for name, loc in zip(names, locs):
    data = sio.loadmat(loc + 'data/iterative_min_RPI.mat')
    min_RPI[name] = {'r': data['H'].shape[0], 'comp_time': float(data['comp_time']/60)}
print(min_RPI)


""" Algorithm approximate max RPI """
max_RPI = {}
for name, loc in zip(names, locs):
    data = sio.loadmat(loc + 'data/iterative_max_RPI.mat')
    max_RPI[name] = {'r': data['H'].shape[0] - 2, 'comp_time': float(data['comp_time']/60)}
print(max_RPI)
