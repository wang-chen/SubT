# Copyright <2020> <Chen Wang [https://chenwang.site], Carnegie Mellon University>

# Redistribution and use in source and binary forms, with or without modification, are 
# permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of 
# conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list 
# of conditions and the following disclaimer in the documentation and/or other materials 
# provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be 
# used to endorse or promote products derived from this software without specific prior 
# written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
# SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
# DAMAGE.

import os
import tqdm
import glob
import torch
import argparse
import numpy as np
import torchvision
from PIL import Image
import torch.utils.data as Data
import torchvision.transforms as transforms
from torch.utils.data import Dataset, DataLoader

class SubT(Dataset):
    '''
    The DARPA Subterranean (SubT) Challenge Front camera data recorded by Team Exploer
    args:
    root: dataset location (without subt-front)
    train: bool value
    test_data: test_data id [0-6], ignored if train=True
    '''
    data = ['0817-ugv0-tunnel0',
            '0817-ugv1-tunnel0',
            '0818-ugv0-tunnel1',
            '0818-ugv1-tunnel1',
            '0820-ugv0-tunnel1',
            '0821-ugv0-tunnel0',
            '0821-ugv1-tunnel0']

    def __init__(self, root, train=True, test_data=0, transform=None):
        super().__init__()
        self.transform, self.train = transform, train
        if train is True:
            self.filenames = sorted(glob.glob(os.path.join(root, 'SubTF', 'train/*.png')))
        else:
            self.filenames = sorted(glob.glob(os.path.join(root, 'SubTF', self.data[test_data], '*.png')))
        self.nframes = len(self.filenames)

    def __len__(self):
        return self.nframes

    def __getitem__(self, idx):
        frame = Image.open(self.filenames[idx])
        if self.transform is not None:
            frame = self.transform(frame)
        return frame

def show_batch(batch, name='video', waitkey=1):
    min_v = torch.min(batch)
    range_v = torch.max(batch) - min_v
    if range_v > 0:
        batch = (batch - min_v) / range_v
    else:
        batch = torch.zeros(batch.size())
    grid = torchvision.utils.make_grid(batch, padding=0).cpu()
    img = grid.numpy()[::-1].transpose((1, 2, 0))
    cv2.imshow(name, img)
    cv2.waitKey(waitkey)
    return img


if __name__ == "__main__":
    '''
    Usage smaple of the SubTF dataset
    '''
    import cv2

    parser = argparse.ArgumentParser(description='Networks')
    parser.add_argument("--data-root", type=str, default='/data/datasets', help="dataset root folder")
    args = parser.parse_args(); print(args)

    transform = transforms.Compose([
        transforms.Resize((320,320)),
        transforms.ToTensor()])

    data = SubT(root=args.data_root, train=False, transform=transform, test_data=1)
    loader = Data.DataLoader(dataset=data, batch_size=1, shuffle=False)

    for batch_idx, frame in enumerate(tqdm.tqdm(loader)):
        show_batch(frame)
    print('Done.')
