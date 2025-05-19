# Traffic Sign Detection with Mask R-CNN

This project implements a Mask R-CNN-based traffic sign detection and classification system for Vietnamese traffic signs.

## Features

- Uses Mask R-CNN architecture with ResNet-50 backbone
- Calculates IoU (Intersection over Union) for better evaluation
- Reports class-wise prediction accuracy
- Supports 52 Vietnamese traffic sign classes

## Prerequisites

- Python 3.8+
- PyTorch 1.10+
- torchvision
- torchmetrics
- OpenCV
- NumPy
- Pandas
- Matplotlib

## Dataset Structure

The code expects the dataset to be organized in the following structure:

```
data/traffic-sign-vietnamese/archive/
├── images/
│   ├── image1.jpg
│   ├── image2.jpg
│   └── ...
├── labels/
│   ├── image1.txt
│   ├── image2.txt
│   └── ...
└── split_dataset/
    ├── train_files.txt
    └── test_files.txt
```

## Usage

1. Place your dataset in the expected directory structure
2. Run the training script:

```bash
python train_maskrcnn.py
```

The script will:
- Create a Mask R-CNN model
- Train it on the traffic sign dataset
- Calculate and display IoU metrics
- Print class-wise prediction accuracy
- Save the trained model in the `checkpoints` directory

## Implementation Details

The implementation includes:

- `mask_rcnn_traffic_sign.py`: Main implementation with dataset, model, training, and evaluation code
- `train_maskrcnn.py`: Script to run the training process

## Key Metrics

The system reports several key metrics:
- Mean Average Precision (mAP@0.5:0.95)
- Mean Average Precision at IoU threshold 0.5 (mAP@0.5)
- Average IoU for predictions with score above a threshold
- Class-wise accuracy and precision

## Class-wise Accuracy

After training, the system reports detailed class-wise performance including:
- Accuracy: Proportion of ground truth instances correctly detected
- Precision: Proportion of predictions that are correct
- Count of correct detections, total ground truth instances, and total predictions 