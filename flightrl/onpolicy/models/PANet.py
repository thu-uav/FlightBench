import torch
import torch.nn as nn
import torch.nn.functional as F

# import pretrained-encoder
class PANet(nn.Module):
    def __init__(self):
        super(PANet, self).__init__()
        self.embedding_size = 64
        self.conv_layer = nn.Sequential(
            # input will be resize into (bs, 1, 112, 112)
            nn.Conv2d(in_channels = 1, out_channels = 4, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.Conv2d(in_channels = 4, out_channels = 4, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            # (bs, 4, 56, 56)
            nn.Conv2d(in_channels = 4, out_channels = 8, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.Conv2d(in_channels = 8, out_channels = 8, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            # (bs, 8, 28, 28)
            nn.Conv2d(in_channels = 8, out_channels = 16, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.Conv2d(in_channels = 16, out_channels = 16, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            # (bs, 16, 14, 14)
            nn.Conv2d(in_channels = 16, out_channels = 16, kernel_size = 1, stride = 1, padding = 0)
            # 1*1 conv
        )
        self.output_layer = nn.Sequential(
            nn.Linear(in_features=16*14*14, out_features=self.embedding_size)
        )
        self.cmd_mlp = nn.Sequential(
            nn.Linear(in_features=self.embedding_size + 9 + 3, out_features=256),
            nn.ReLU(),
            nn.Linear(in_features=256, out_features=128),
            nn.ReLU(),
            nn.Linear(in_features=128, out_features=64),
            nn.ReLU(),
            nn.Linear(in_features=64, out_features=4)
        )
    
    def forward(self, depth, imu):
        depth = self.conv_layer(depth)
        depth = depth.reshape(-1, 16*14*14)
        depth_output = self.output_layer(depth)
        # output = depth_output.detach().cpu().numpy()[0]
        # print(output)
        out = self.cmd_mlp(torch.cat((depth_output, imu), dim = 1))
        return out
    
    def encoder_forward(self, x): # x is a depth of (bs, 1, 112, 112)
        x = self.conv_layer(x)
        x = x.reshape(-1, 16*14*14)
        x = self.output_layer(x)
        return x
    
class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()
        self.embedding_size = 64
        self.decode_layer = nn.Sequential(
            nn.Linear(in_features=self.embedding_size, out_features=4096),
            nn.ReLU(),
            nn.Linear(in_features=4096, out_features=8192),
            nn.ReLU(),
            nn.Linear(in_features=8192, out_features=112*112),
            nn.ReLU(),
        )
    
    def forward(self, x):
        x = self.decode_layer(x)
        x = x.reshape(-1, 1, 112, 112)
        return x