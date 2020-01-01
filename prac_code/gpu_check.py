#Tensorflow and Pytorch CUDA and GPU Check
from __future__ import print_function
import tensorflow as tf
from keras import backend as K
hello = tf.constant("Hello World")

tf.print(hello)

import torch
if torch.cuda.is_available():
	print('PyTorch running on Nvidia Geforce GT920M with CUDA Toolkit 10.0.130')
else:
	print('CUDA unavailable, PyTorch running on CPU')

x = torch.empty(5, 3)

print(x)

#end of code