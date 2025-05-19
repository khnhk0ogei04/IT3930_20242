import os
import sys
import torch
from mask_rcnn_traffic_sign import main

if __name__ == "__main__":
    # Check if CUDA is available
    if torch.cuda.is_available():
        print(f"GPU available: {torch.cuda.get_device_name(0)}")
    else:
        print("No GPU available, running on CPU")
    
    # Create directories if they don't exist
    os.makedirs("./data/traffic-sign-vietnamese/archive/", exist_ok=True)
    os.makedirs("./checkpoints", exist_ok=True)
    
    # Run the main training function
    main() 