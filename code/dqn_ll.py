# Deep Q-Network for Lunar Lander
# Function Approximation and TD-Learning using Deep Neural Networks

import numpy as np
import time
import random
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

device = T.device("cuda" if T.cuda.is_available() else "cpu")

class DQNet(nn.Module):
    def __init__(self,alpha):
        super(DQNet, self).__init__()
