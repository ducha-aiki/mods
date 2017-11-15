#!/usr/bin/python2 -utt
# -*- coding: utf-8 -*-
import sys
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import torch.backends.cudnn as cudnn
import time
import os
sys.path.insert(0, '/home/ubuntu/dev/opencv-3.1/build/lib')
import cv2
import math
import numpy as np
from architectures import OriNet, YiNet,AffNetPlainOld,AffNetFast,AffNetFasterELUMP,TNet
from HandCraftedModules import OrientationDetector,AffineShapeEstimator

PSS = 32
USE_CUDA = True

#model = AffNetPlainOld(PS = PSS)
#weightd_fname = '/home/old-ufo/dev/learning_ori/affnetplain16.pth'# very good

model = AffNetFast(PS = PSS)#0.000 298 ms
#Graffity
#weightd_fname = '/home/old-ufo/dev/vlbenchmakrs-1.0-beta/data/software/MODSHesAff/new6b_10M005_9.pth' #  also very good
weightd_fname = '/home/old-ufo/dev/mods/wxbs-journal/build/newHP6b_10M005_04.pth'
#weightd_fname = '/home/old-ufo/dev/mods/wxbs-journal/build/new6b_10M005_5.pth'
#weightd_fname = '/home/old-ufo/dev/oxford-affine/new_weights/affnetfast_3b_no_shift_17ep.pth' # 25 / 126  good
#weightd_fname = 'affnetfast_3b_no_shift_18ep.pth' # 26 / 126 , 0.00028

#model = AffNetFasterELUMP(PS = PSS)
#weightd_fname = 'AffNetFasterELUMP12.pth' #30/136, faster 0.000248

#model = TNet(PS = PSS)
#weightd_fname = 'TNet17.pth' #34/161, 0.000234 per patch



checkpoint = torch.load(weightd_fname)
model.load_state_dict(checkpoint['state_dict'])

#model = AffineShapeEstimator(patch_size = PSS) #35/160

model.eval()
if USE_CUDA:
    model.cuda()

try:
    input_img_fname = sys.argv[1]
    output_fname = sys.argv[2]
except:
    print "Wrong input format. Try ./extract_hardnet_desc_from_hpatches_file.py imgs/ref.png out.txt"
    sys.exit(1)

image = cv2.imread(input_img_fname,0)
h,w = image.shape
print(h,w, weightd_fname)

n_patches =  h/w


descriptors_for_net = np.zeros((n_patches, 4))

patches = np.ndarray((n_patches, 1, PSS, PSS), dtype=np.float32)
for i in range(n_patches):
    patch =  image[i*(w): (i+1)*(w), 0:w]
    patches[i,0,:,:] = cv2.resize(patch,(PSS,PSS)) / 255.
bs = 128;
outs = []
n_batches = n_patches / bs + 1
t = time.time()
for batch_idx in range(n_batches):
    if batch_idx == n_batches - 1:
        if (batch_idx + 1) * bs > n_patches:
            end = n_patches
        else:
            end = (batch_idx + 1) * bs
    else:
        end = (batch_idx + 1) * bs
    if batch_idx * bs >= end:
        continue
    data_a = patches[batch_idx * bs: end, :, :, :].astype(np.float32)
    data_a = torch.from_numpy(data_a)
    if USE_CUDA:
        data_a = data_a.cuda()
    data_a = Variable(data_a, volatile=True)
    # compute output
    out_a = model(data_a)
    descriptors_for_net[batch_idx * bs: end,:] = out_a.data.cpu().numpy().reshape(-1, 4)
print descriptors_for_net.shape
et  = time.time() - t
print 'processing', et, et/float(n_patches), ' per patch'
np.savetxt(output_fname,  descriptors_for_net, delimiter=' ', fmt='%10.5f')    