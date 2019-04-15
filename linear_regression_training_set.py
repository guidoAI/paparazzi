# -*- coding: utf-8 -*-
"""
Perform linear regression on a training set stored by the optical flow avoidance module.

Guido de Croon
"""

import numpy as np
from matplotlib import pyplot as plt
import scipy.stats as st
from sklearn import tree
from sklearn.neural_network import MLPRegressor
import graphviz
import os

plt.close('all');

def density_plot(X, title='2D Gaussian Kernel density estimation'):
    # Extract x and y
    x = X[:, 0]
    y = X[:, 1]
    
    # Define the borders
    deltaX = (max(x) - min(x))/10
    deltaY = (max(y) - min(y))/10
    
    xmin = min(x) - deltaX
    xmax = max(x) + deltaX
    
    ymin = min(y) - deltaY
    ymax = max(y) + deltaY
    
    #print(xmin, xmax, ymin, ymax)
    
    # Create meshgrid
    xx, yy = np.mgrid[xmin:xmax:100j, ymin:ymax:100j]
    
    positions = np.vstack([xx.ravel(), yy.ravel()])
    values = np.vstack([x, y])
    kernel = st.gaussian_kde(values)
    f = np.reshape(kernel(positions).T, xx.shape)
    
    fig = plt.figure(figsize=(8,8))
    ax = fig.gca()
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    cfset = ax.contourf(xx, yy, f, cmap='coolwarm')
    ax.imshow(np.rot90(f), cmap='coolwarm', extent=[xmin, xmax, ymin, ymax])
    cset = ax.contour(xx, yy, f, colors='k')
    #cset.levels /= X.shape[0];
    #ax.clabel(cset, inline=1, fontsize=10)
    plt.plot([xmin, xmax], [xmin, xmax], 'r')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.title(title)
    


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
str_title = 'Linear fit';
plt.title(str_title);

density_plot(np.concatenate((y, covs), axis=1), title=str_title);
plt.figure();
plt.hist2d(np.reshape(y, (len(y),)), np.reshape(covs, (len(covs),)), bins=25);
plt.plot([min(y), max(y)], [min(covs), max(covs)], 'r')
plt.title(str_title);

for weight in x:
    print('{0:3.3f} '.format(weight[0]), end='')
    
print('MAE Linear fit = {}'.format(np.mean(abs(y-covs))));
    
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
NN = MLPRegressor(hidden_layer_sizes=(30,10,3), tol=1E-9, random_state = 9, activation='relu', solver='adam', alpha=0.00001, learning_rate='adaptive', max_iter=100000, verbose = True);
NN.fit(textons, covs.ravel());
y = NN.predict(textons);

plt.figure();
plt.plot(y, covs, 'ro')
plt.xlabel('Estimated cov divs');
plt.ylabel('Actual cov divs');
str_title = 'MLP';
plt.title(str_title);

density_plot(np.concatenate((y.reshape(covs.shape), covs), axis=1), title=str_title);
plt.figure();
plt.hist2d(np.reshape(y, (len(y),)), np.reshape(covs, (len(covs),)), bins=25);
plt.plot([min(y), max(y)], [min(y), max(y)], 'r');
plt.title(str_title);
print('MAE MLP = {}'.format(np.mean(abs(y-covs))));
print('MSE MLP = {}'.format(np.mean(np.multiply(y-covs, y-covs))));