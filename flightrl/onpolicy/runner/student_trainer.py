import os

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import numpy as np
import cv2
from PIL import Image

from onpolicy.models.PANet import PANet, Decoder
import matplotlib
import matplotlib.pyplot as plt

class PADataset(Dataset):
    def __init__(self, data_dir):
        super(PADataset, self).__init__()
        self.imgs = []
        self.imu = []
        self.len = 0
        for folder in os.listdir(data_dir):
            if folder != ".." and folder != ".":
                files = os.listdir(os.path.join(data_dir, folder))
                if len(files) % 2 != 0:
                    print("data invalid")
                    exit(-1)
                data_num = len(files) // 2
                for i in range(data_num):
                    fname = str(i)+'.npy'
                    img = np.load(os.path.join(data_dir, folder, fname))
                    # print("test shape: ", img.shape)
                    # print("test range: max ", np.max(img), " min ", np.min(img))
                    # if np.max(img) < 0.1:
                    # # fig_name = str(i)+'.png'
                    # # matplotlib.image.imsave(os.path.join(data_dir, "image", fig_name), img)
                    img = cv2.resize(img, (112, 112))
                    img = img.reshape(1, 112, 112)
                    # print("img: ", img.shape)
                    self.imgs.append(img.copy())
                    fname = str(i)+'.txt'
                    imu = np.loadtxt(os.path.join(data_dir, folder, fname))
                    # print("imu: ", imu.shape)
                    self.imu.append(imu.copy())
                self.len += data_num
        if self.len != len(self.imu) or len(self.imu)!=len(self.imgs):
            print("load data invalid!")
            exit(-1)
        self.imgs = np.stack(self.imgs, dtype = np.float32)
        self.imu = np.stack(self.imu, dtype = np.float32)
        print("data set with", self.imgs.shape[0], "samples built!")

    def __len__(self):
        return self.len

    def __getitem__(self, index):
        depth = self.imgs[index]
        imu = np.zeros(9+3, dtype=np.float32)
        quat = self.imu[index, 3:7]
        R = np.array(
        [[1.0 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]), 2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * (quat[0] * quat[2] + quat[1] * quat[3])],
         [2 * (quat[1] * quat[2] + quat[0] * quat[3]), 1.0 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]), 2 * (quat[2] * quat[3] - quat[0] * quat[1])],
         [2 * (quat[1] * quat[3] - quat[0] * quat[2]), 2 * (quat[2] * quat[3] + quat[0] * quat[1]), 1.0 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])]])
        imu[:9] = R.reshape(9)
        imu[9:12] = self.imu[index, 7:10]
        label = self.imu[index, 13:17]
        return depth, imu, label

class Trainer():
    def __init__(self, data_dir, eval_dir, pretrain_dir = None, lr = 0.0004):
        self.data_dir = data_dir
        self.eval_dir = eval_dir
        self.net = PANet()
        self.decoder = Decoder()
        if pretrain_dir != None:
            print("load PAnet from ", pretrain_dir)
            self.net.load_state_dict(torch.load(pretrain_dir, map_location=torch.device('cpu')))
        if torch.cuda.is_available():
            print("use gpu")
            self.device = torch.device("cuda:0")
            self.net.to(self.device)
            self.decoder.to(self.device)

        params_for_autoencoder = []
        for name, pa in self.net.named_parameters():
            if "cmd" not in name:
                params_for_autoencoder.append(pa)
        for name, pa in self.decoder.named_parameters():
            params_for_autoencoder.append(pa)
        self.encoder_optimizer = torch.optim.Adam(params_for_autoencoder, lr = lr)

        params_for_action = []
        for name, pa in self.net.named_parameters():
            if "cmd" in name:
            # if True:
                params_for_action.append(pa)
        self.action_optimizer = torch.optim.Adam(params_for_action, lr = lr)
    
    def make_dataset(self):
        self.dataset = PADataset(data_dir=self.data_dir)
        self.eval_dataset = PADataset(data_dir=self.eval_dir)

    def train_autoencoder(self, lr = 0.001, epoch = 100, bs = 512):
        loss_func = nn.MSELoss()
        dataloader = DataLoader(self.dataset, batch_size=bs, shuffle=True)
        eval_dataloader = DataLoader(self.eval_dataset, batch_size=bs, shuffle=True)
        ep_train_loss_list = []
        ep_eval_loss_list = []
        for ep in range(epoch):
            loss_list = []
            eval_loss_list = []
            for i, (depth, imu, label) in enumerate(dataloader):
                depth_use = depth.to(self.device)
                self.encoder_optimizer.zero_grad()
                encode_res = self.net.encoder_forward(depth_use)
                reconstruct_res = self.decoder(encode_res)
                loss = loss_func(depth_use, reconstruct_res)
                loss_list.append(loss.item())
                loss.backward()
                self.encoder_optimizer.step()
            for i, (depth, imu, label) in enumerate(eval_dataloader):
                depth_use = depth.to(self.device)
                encode_res = self.net.encoder_forward(depth_use)
                reconstruct_res = self.decoder(encode_res)
                loss = loss_func(depth_use, reconstruct_res)
                eval_loss_list.append(loss.item())
                depth_save = depth_use.detach().cpu().numpy()[0, 0, :, :]
                # print(reconstruct_res.shape)
                depth_reconstruct = reconstruct_res.detach().cpu().numpy()[0].reshape(112, 112)
                matplotlib.image.imsave("origin.png", depth_save)
                matplotlib.image.imsave("reconstruct.png", depth_reconstruct)
            print("epoch "+str(ep+1)+"/"+str(epoch)+", loss: ", sum(loss_list) / len(loss_list), ", eval_loss: ", sum(eval_loss_list) / len(eval_loss_list))
            ep_train_loss_list.append(sum(loss_list) / len(loss_list))
            ep_eval_loss_list.append(sum(eval_loss_list) / len(eval_loss_list))
        # plt.figure()
        # plt.plot(ep_train_loss_list, label = "train_loss")
        # plt.plot(ep_eval_loss_list, label = "eval_loss")
        # plt.legend()
        # plt.savefig('./train_encoder_loss_curve.png')

    def save(self, save_dir = "./PAnet.pt"):
        torch.save(self.net.state_dict(), save_dir)

    def train_action(self, lr = 0.001, epoch = 100, bs = 512):
        loss_func = nn.MSELoss()
        dataloader = DataLoader(self.dataset, batch_size=bs, shuffle=True)
        eval_dataloader = DataLoader(self.eval_dataset, batch_size=bs, shuffle=True)
        ep_train_loss_list = []
        ep_eval_loss_list = []
        for ep in range(epoch):
            loss_list = []
            eval_loss_list = []
            for i, (depth, imu, label) in enumerate(dataloader):
                depth_use = depth.to(self.device)
                imu_use = imu.to(self.device)
                label_use = label.to(self.device)
                self.action_optimizer.zero_grad()
                action_res = self.net(depth_use, imu_use)
                loss = loss_func(action_res, label_use)
                loss_list.append(loss.item())
                loss.backward()
                self.action_optimizer.step()
            for i, (depth, imu, label) in enumerate(eval_dataloader):
                depth_use = depth.to(self.device)
                imu_use = imu.to(self.device)
                label_use = label.to(self.device)
                action_res = self.net(depth_use, imu_use)
                loss = loss_func(action_res, label_use)
                eval_loss_list.append(loss.item())
                print("label_action: ", label[0])
                print("learned_action: ", action_res[0])

            print("epoch "+str(ep+1)+"/"+str(epoch)+", loss: ", sum(loss_list) / len(loss_list), ", eval_loss: ", sum(eval_loss_list) / len(eval_loss_list))
            ep_train_loss_list.append(sum(loss_list) / len(loss_list))
            ep_eval_loss_list.append(sum(eval_loss_list) / len(eval_loss_list))
        # plt.figure()
        # plt.plot(ep_train_loss_list, label = "train_loss")
        # plt.plot(ep_eval_loss_list, label = "eval_loss")
        # plt.legend()
        # plt.savefig('./train_action_loss_curve.png')

def main():
    # pre train with offline data
    # trainer = Trainer(data_dir="/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/data", eval_dir="/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/eval", pretrain_dir="/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/runner/PAnet.pt")
    trainer = Trainer(data_dir="/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/offline_data", eval_dir="/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/eval", pretrain_dir=None)
    trainer.make_dataset()
    trainer.train_autoencoder(lr = 0.0005, epoch = 500, bs = 6144)
    trainer.save()
    trainer.train_action(lr = 0.0004, epoch = 500, bs = 6144)
    trainer.save()

    # dataset = PADataset(data_dir="/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/data")
    # dataloader = DataLoader(dataset, batch_size=512, shuffle=True)
    # for i, (depth, imu, label) in enumerate(dataloader):
    #     print("i: ", i)
    #     print("depth: ", depth.shape)
    #     print("imu: ", imu.shape)
    #     print("label: ", label.shape)

if __name__ == "__main__":
    main()

