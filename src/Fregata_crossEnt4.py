
#Import
from math import sin, cos, pi
from scipy import eye, matrix, random, asarray
import pdb
from pybrain.rl.environments.graphical import GraphicalEnvironment
import numpy as np
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation
from keras.regularizers import l2
import keras
from matplotlib import pyplot as plt
import theano
import theano.tensor as T
import math
import h5py


print "Loading Data"
f       = h5py.File("/home/sisl/kyle/AA241X/Fregata_TrainingData_Speed_72BTARGET.h5",'r')
X_train = np.array(f['X'])
Q       = np.array(f['y'])
print "Setting up Model"

#def both(y_true, y_pred):
#    d = y_true-y_pred
#    mins = T.argmin(y_true,axis=1)
#    mins_onehot = T.extra_ops.to_one_hot(mins,5)
#    others_onehot = mins_onehot-1
#    d_opt = d*mins_onehot #+ d*others_onehot
#    d_sub = d*others_onehot
#    a = 20*d_opt**2
#    b = d_opt**2
#    c = 5*d_sub**2
#    d = d_sub**2
#    l = T.switch(d_sub<0,c,d) + T.switch(d_opt<0,a,b)
#    cost1 = l
#
#    return cost1

model = Sequential()

model.add(Dense(20, input_dim=5, init='uniform', activation='relu'))
model.add(Dense(20, init='uniform', activation='relu'))
model.add(Dense(20, init='uniform', activation='relu'))
model.add(Dense(20, init='uniform', activation='relu'))
model.add(Dense(2,   init='uniform', activation='softmax'))

model.compile(loss='categorical_crossentropy', optimizer='adamax')

#model.load_weights("/home/sisl/kyle/data/ACASXU_ADAMAX_customLoss_900_norm.h5")

cb = keras.callbacks.ModelCheckpoint("/home/sisl/kyle/AA241X/data/FREGAT2_Speed_72BTARGET_20.h5", verbose=1, save_best_only=False)

print "loaded and ready to train"

for i in range(3):
    model.fit(X_train, Q, nb_epoch=300, batch_size=2**16,shuffle=True, show_accuracy=True, callbacks=[cb])
    
    fileH5 = "/home/sisl/kyle/AA241X/data/FREGAT2_Speed_72BTARGET_20_%d.h5" % ((i+1)*300)
    model.save_weights(fileH5,overwrite=True)


