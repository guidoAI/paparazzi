# -*- coding: utf-8 -*-
"""
Read a dictionary saved by the textons module.

Guido de Croon
"""

import numpy as np
from matplotlib import pyplot as plt

f = open('Dictionary_00000.dat', 'r');
array = []
for line in f: # read rest of lines
    array.append([float(x) for x in line.split()])
f.close();
array = np.asarray(array);

# params
n_textons = 20; # number of visual words
patch_size = 6; # size of one patch
n_channels = 3;

# extract YUV components
n_elements = len(array);
inds_U = np.arange(0, n_elements, 4);
U_ALL = array[inds_U];
inds_Y1 = np.arange(1, n_elements, 4);
Y1_ALL = array[inds_Y1];
inds_V = np.arange(2, n_elements, 4);
V_ALL = array[inds_V];
inds_Y2 = np.arange(3, n_elements, 4);
Y2_ALL = array[inds_Y2];

Dictionary = np.zeros([n_textons, patch_size*patch_size*n_channels]);

n_plots = int(np.ceil(np.sqrt(n_textons)));
fig, _axs = plt.subplots(nrows=n_plots, ncols=n_plots);

rgbimg = np.zeros([patch_size, patch_size, 3]);

for w in range(n_textons):
    
    # extract YUV per texton
    U = U_ALL[np.arange(w*patch_size*n_channels, (w+1)*patch_size*n_channels)];
    Y1 = Y1_ALL[np.arange(w*patch_size*n_channels, (w+1)*patch_size*n_channels)];
    V = V_ALL[np.arange(w*patch_size*n_channels, (w+1)*patch_size*n_channels)];
    Y2 = Y2_ALL[np.arange(w*patch_size*n_channels, (w+1)*patch_size*n_channels)];
    
    # conversion
    R1 = Y1 + 1.4022 * (V - 128);
    G1 = Y1 - 0.3456 * (U - 128) - (0.7145 * (V - 128));
    B1 = Y1 + 1.7710 * (U - 128);
    R2 = Y2 + 1.4022 * (V - 128);
    G2 = Y2 - 0.3456 * (U - 128) - (0.7145 * (V - 128));
    B2 = Y2 + 1.7710 * (U - 128);
    
    R = np.zeros([patch_size,patch_size]);
    G = np.zeros([patch_size,patch_size]);
    B = np.zeros([patch_size,patch_size]);
    R[:,0::2] = np.transpose(np.reshape(R1,[3,6], order='F'));
    R[:,1::2] = np.transpose(np.reshape(R2,[3,6], order='F'));
    G[:,0::2] = np.transpose(np.reshape(G1,[3,6], order='F'));
    G[:,1::2] = np.transpose(np.reshape(G2,[3,6], order='F'));
    B[:,0::2] = np.transpose(np.reshape(B1,[3,6], order='F'));
    B[:,1::2] = np.transpose(np.reshape(B2,[3,6], order='F'));
    
    # clip the values into range [0, 255]
    R = np.maximum(np.minimum(R, 255), 0);
    G = np.maximum(np.minimum(G, 255), 0);
    B = np.maximum(np.minimum(B, 255), 0);

    
    # form bgr image
    R_norm = R / 255.0;
    G_norm = G / 255.0;
    B_norm = B / 255.0;
    rgbimg[:,:,0] = B_norm;
    rgbimg[:,:,1] = G_norm;
    rgbimg[:,:,2] = R_norm;
    
    _axs[int(np.floor(w / n_plots)), int(np.mod(w, n_plots))].imshow(rgbimg);
    im_label = 'Texton ' + str(w+1)
    _axs[int(np.floor(w / n_plots)), int(np.mod(w, n_plots))].set_title(im_label)
    
#    Dictionary(w,1:patch_size*patch_size) = reshape(rgbimg(:,:,1)',1,36);
#    Dictionary(w,patch_size*patch_size+1:patch_size*patch_size*2) = reshape(rgbimg(:,:,2)',1,36);
#    Dictionary(w,patch_size*patch_size*2+1:patch_size*patch_size*3) = reshape(rgbimg(:,:,3)',1,36);

