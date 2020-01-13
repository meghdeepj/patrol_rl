#Tensorflow and Pytorch CUDA and GPU Check
from __future__ import print_function
import tensorflow as tf
from keras import backend as K
hello = tf.constant("Hello World")

tf.print(hello)

import torch
if torch.cuda.is_available():
	print('PyTorch running on Nvidia Geforce GT920M with CUDA Toolkit 10.0.130')
	print(torch.cuda.get_device_capability(0))
else:
	print('CUDA unavailable, PyTorch running on CPU')

gpu=torch.device('cuda')
x = torch.empty(5, 3)
y=torch.rand(4,3, device=gpu)
print(x,y)

#end of code