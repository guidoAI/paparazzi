# -*- coding: utf-8 -*-
"""
Perform linear regression on a training set stored by the optical flow avoidance module.

Guido de Croon
"""

import numpy as np
from matplotlib import pyplot as plt
from sklearn import tree
from sklearn.neural_network import MLPRegressor
import graphviz
import os

f = open('Training_set_00000.dat', 'r');
array = []
for line in f: # read rest of lines
    array.append([float(x) for x in line.split()])
f.close();
array = np.asarray(array);

# params
gain_id = 0;
cov_id = 1;
class_id = 2;
textons_start_id = 3;
n_textons = 20; # number of visual words


# extract the different components
n_elements = len(array);
gains = array[:, gain_id];
covs = array[:, cov_id];
classes = array[:, class_id];
textons = array[:, textons_start_id:textons_start_id+n_textons];
A_bias = np.ones([n_elements, 1]);
A = np.concatenate((textons, A_bias), axis=1);

covs = covs.reshape([n_elements, 1]);
# mqximum likelihood:
# x = np.linalg.pinv(A) @ covs;
# maximum a posteriori:
prior = 0.0;
AT = np.transpose(A);
ATA = AT @ A;
ATA += prior * np.eye(ATA.shape[0]);
x = (np.linalg.inv(ATA) @ AT) @ covs;

# predict:
y = A @ x;

plt.figure();
plt.plot(y, covs, 'o')
plt.xlabel('Estimated cov divs');
plt.ylabel('Actual cov divs')
plt.title('Linear fit');

for weight in x:
    print('{0:3.3f} '.format(weight[0]), end='')
    
    
clf = tree.DecisionTreeRegressor(max_depth=4)
clf = clf.fit(textons, covs);
y = clf.predict(textons);
dot_data = tree.export_graphviz(clf, out_file='tree.dot', filled=True, rounded=True, special_characters=True)  
graph = graphviz.Source(dot_data);
# dot -Tpng tree.dot -o tree.png
os.system("dot -Tpng tree.dot -o tree.png")

plt.figure();
plt.plot(y, covs, 'go', )
plt.xlabel('Estimated cov divs');
plt.ylabel('Actual cov divs')
plt.title('Decision tree');


#NN = MLPRegressor(hidden_layer_sizes=(100,), activation=’relu’, solver=’adam’, alpha=0.0001, batch_size=’auto’, learning_rate=’constant’, learning_rate_init=0.001, max_iter=200);
NN = MLPRegressor(hidden_layer_sizes=(30,),activation='relu', alpha=0.01, max_iter=1000);
NN = NN.fit(textons, covs.ravel());
y = NN.predict(textons);

plt.figure();
plt.plot(y, covs, 'ro')
plt.xlabel('Estimated cov divs');
plt.ylabel('Actual cov divs')
plt.title('MLP');
