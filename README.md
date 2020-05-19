# SubT

The SubT dataset PyTorch dataloader.

# Description

   Refer to [The AirLab Dataset Instructions](http://theairlab.org/dataset/interestingness)

# Dataset Usage example in PyTorch:

        data = SubTF(root='/data/location', train=False, transform=transform, test_data=1)
        loader = Data.DataLoader(dataset=data, batch_size=1, shuffle=False)

# Evaluation Tools

For single sequence:

## Usage Example

We provide results example files in folder 'results-example'.

For single sequence:

      python evaluation.py --source 'ground-truth/0817-ugv0-tunnel0-interest-1.txt' --target 'results-example/SubTF-0-2020-03-04-21:43:27-example.txt'
      # source: ground-truth file
      # target: results file

For overall performance on entire dataset

      python performance.py --save-flag example --root results-example --category interest-1
      # save-flag: ground file
      # results-example: location of results file
      # category: interest-1 or interest-2

More information about this evaluation metric, checkout [our paper](https://arxiv.org/pdf/2005.08829.pdf).


# Citation

      @article{wang2020visual,
         title={Visual Memorability for Robotic Interestingness via Unsupervised Online Learning},
         author={Wang, Chen and Wang, Wenshan and Qiu, Yuheng and Hu, Yafei and Scherer, Sebastian},
         journal={arXiv preprint arXiv:2005.08829},
         year={2020}
         }
