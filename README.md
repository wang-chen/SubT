# SubTF

The SubTF dataset PyTorch dataloader.

# Description

   Refer to [The AirLab Dataset Instructions](http://theairlab.org/dataset/interestingness)

# Usage example:

        data = SubTF(root=args.data_root, train=False, transform=transform, test_data=1)
        loader = Data.DataLoader(dataset=data, batch_size=1, shuffle=False)
# Project

   Refer to [The AirLab Project Description](http://theairlab.org/interestingness)
