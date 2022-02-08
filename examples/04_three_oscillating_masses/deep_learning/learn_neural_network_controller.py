import numpy as np
import scipy.io as sio
import tensorflow as tf
from tensorflow import keras
import pdb


""" Params """
# network
n_hidden_layers = 2
w_hidden_layers = 20

# training
batch_size = 256
epochs = 5000


""" Load data """
act = 'relu'
data = sio.loadmat('../data/data_learning.mat')
X = data['X']
U = data['U']


""" Build network model """
inputs = keras.Input(shape=(X.shape[0],))
x = keras.layers.Dense(w_hidden_layers,activation=act)(inputs)
for _ in range(n_hidden_layers-1):
    x = keras.layers.Dense(w_hidden_layers,activation=act)(x)
outputs = keras.layers.Dense(U.shape[0],activation='linear')(x)

model = keras.Model(inputs, outputs)

model.compile(optimizer = 'adam', loss = 'mse')


""" Fit model """
hist = model.fit(X.T, U.T, batch_size = batch_size, epochs = epochs, shuffle = True)


""" Save results """
model.save('../data/nn_controller.h5')
exp_dic = {'n_hl': n_hidden_layers, 'act': act}
weights = []
biases = []
# extract weights and biases
for i in range(n_hidden_layers + 1):
    weights.append(model.weights[2 * i].numpy().T)
    biases.append(np.reshape(model.weights[2 * i + 1].numpy(), (-1, 1)))
network = {'weights': weights, 'biases': biases}
exp_dic = {'network': network}
sio.savemat('../data/nn_controller.mat', exp_dic)
