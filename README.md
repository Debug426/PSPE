3D Shape Part Segmentation
============================
<img src="../figure/HumanPoseEstimation.png" width="500" />

## Installation

### Requirements
* Hardware: GPUs to hold 14000MB
* Software: 
  Linux (tested on Ubuntu 18.04)
  PyTorch>=1.5.0, Python>=3, CUDA>=10.1, tensorboardX, tqdm, pyYaml

### Dataset
Download and HRHP Dataset: [HRHP](https://1drv.ms/f/s!Au5BcTkJGU2pklK7ELBlAELQl9dH?e=GjwA5Z) 
``` 
mkdir -p data
ln -s /path to shapenet part/data/human
```

## Usage

* Train:

   * Multi-thread training :

     * `python main.py --config config/dgcnn_paconv_train.yaml` 
  
* Visualization: [tensorboardX](https://github.com/lanpa/tensorboardX) incorporated for better visualization.

   `tensorboard --logdir=checkpoints/exp_name`

* Human joints regconized:  

   * Running ./HJR/Humanjoint.py you will get all joints locations whicie save as human_joint.txt. (you need to change the save path at line 180)
   * Running ./HJR/Humanjoint.py you will also see the human point and human joints.

* Human pose Visualization:

   * Put the human_joint.txt into folider ./HJR and run human_point_display.m (Matlab Code) you will see the human pose.

## Contact

You are welcome to send pull requests or share some ideas with us. Contact information: Huanyu Deng(2021935717@qqhru.edu.cn) .

