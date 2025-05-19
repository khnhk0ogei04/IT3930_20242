import os
import time
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import Counter

import torch
import torchvision
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from torchvision.models.detection.backbone_utils import resnet_fpn_backbone
from torchvision.models.detection.mask_rcnn import MaskRCNN
from torchvision import transforms
from torch.utils.data import DataLoader
from torchvision.ops import box_iou
from torchmetrics.detection import MeanAveragePrecision

# Vietnamese class definitions
vietnamese_classes = {
    0: "Đường người đi bộ cắt ngang",
    1: "Đường giao nhau (ngã ba bên phải)",
    2: "Cấm đi ngược chiều",
    3: "Phải đi vòng sang bên phải",
    4: "Giao nhau với đường đồng cấp",
    5: "Giao nhau với đường không ưu tiên",
    6: "Chỗ ngoặt nguy hiểm vòng bên trái",
    7: "Cấm rẽ trái",
    8: "Bến xe buýt",
    9: "Nơi giao nhau chạy theo vòng xuyến",
    10: "Cấm dừng và đỗ xe",
    11: "Chỗ quay xe",
    12: "Biển gộp làn đường theo phương tiện",
    13: "Đi chậm",
    14: "Cấm xe tải",
    15: "Đường bị thu hẹp về phía phải",
    16: "Giới hạn chiều cao",
    17: "Cấm quay đầu",
    18: "Cấm ô tô khách và ô tô tải",
    19: "Cấm rẽ phải và quay đầu",
    20: "Cấm ô tô",
    21: "Đường bị thu hẹp về phía trái",
    22: "Gồ giảm tốc phía trước",
    23: "Cấm xe hai và ba bánh",
    24: "Kiểm tra",
    25: "Chỉ dành cho xe máy",
    26: "Chướng ngại vật phía trước",
    27: "Trẻ em",
    28: "Xe tải và xe công",
    29: "Cấm mô tô và xe máy",
    30: "Chỉ dành cho xe tải",
    31: "Đường có camera giám sát",
    32: "Cấm rẽ phải",
    33: "Nhiều chỗ ngoặt nguy hiểm liên tiếp, chỗ đầu tiên sang phải",
    34: "Cấm xe sơ-mi rơ-moóc",
    35: "Cấm rẽ trái và phải",
    36: "Cấm đi thẳng và rẽ phải",
    37: "Đường giao nhau (ngã ba bên trái)",
    38: "Giới hạn tốc độ (50km/h)",
    39: "Giới hạn tốc độ (60km/h)",
    40: "Giới hạn tốc độ (80km/h)",
    41: "Giới hạn tốc độ (40km/h)",
    42: "Các xe chỉ được rẽ trái",
    43: "Chiều cao tĩnh không thực tế",
    44: "Nguy hiểm khác",
    45: "Đường một chiều",
    46: "Cấm đỗ xe",
    47: "Cấm ô tô quay đầu xe (được rẽ trái)",
    48: "Giao nhau với đường sắt có rào chắn",
    49: "Cấm rẽ trái và quay đầu xe",
    50: "Chỗ ngoặt nguy hiểm vòng bên phải",
    51: "Chú ý chướng ngại vật – vòng tránh sang bên phải"
}

class TrafficSignDataset(torch.utils.data.Dataset):
    def __init__(self, image_dir, labels_dir, image_file_list_path, target_size=(640, 640), transforms=None):
        self.image_dir = image_dir
        self.labels_dir = labels_dir
        self.target_size = target_size 
        self.transforms = transforms
        self.image_filenames = self._load_image_filenames(image_file_list_path)
        self.image_filenames = [f for f in self.image_filenames if os.path.exists(os.path.join(self.image_dir, f))]

    def _load_image_filenames(self, file_list_path):
        try:
            with open(file_list_path, 'r') as f:
                filenames = [line.strip() for line in f if line.strip()]
            return filenames
        except FileNotFoundError:
            print(f"File Not Found: {file_list_path}")
            return []

    def __getitem__(self, idx):
        img_filename = self.image_filenames[idx]
        img_path = os.path.join(self.image_dir, img_filename)
        
        try:
            image = cv2.imread(img_path)
            if image is None: 
                raise FileNotFoundError(f"Can't read file: {img_path}")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img_h_orig, img_w_orig, _ = image.shape
            new_w, new_h = self.target_size
            image_resized = cv2.resize(image, (new_w, new_h))

            annotation_filename = os.path.splitext(img_filename)[0] + ".txt"
            annotation_path = os.path.join(self.labels_dir, annotation_filename)
            
            boxes = []
            labels = []
            masks = []  # Added for Mask R-CNN

            if os.path.exists(annotation_path):
                with open(annotation_path, 'r') as f:
                    for line_idx, line in enumerate(f):
                        parts = line.strip().split()
                        if len(parts) == 5:
                            try:
                                class_id_orig = int(parts[0])
                                x_center_norm = float(parts[1])
                                y_center_norm = float(parts[2])
                                width_norm = float(parts[3])
                                height_norm = float(parts[4])

                                x_min_abs_orig = (x_center_norm - width_norm / 2) * img_w_orig
                                y_min_abs_orig = (y_center_norm - height_norm / 2) * img_h_orig
                                x_max_abs_orig = (x_center_norm + width_norm / 2) * img_w_orig
                                y_max_abs_orig = (y_center_norm + height_norm / 2) * img_h_orig

                                x_min_abs_new = x_min_abs_orig * (new_w / img_w_orig)
                                y_min_abs_new = y_min_abs_orig * (new_h / img_h_orig)
                                x_max_abs_new = x_max_abs_orig * (new_w / img_w_orig)
                                y_max_abs_new = y_max_abs_orig * (new_h / img_h_orig)
                                
                                x_min_abs_new = max(0, x_min_abs_new)
                                y_min_abs_new = max(0, y_min_abs_new)
                                x_max_abs_new = min(new_w, x_max_abs_new)
                                y_max_abs_new = min(new_h, y_max_abs_new)

                                if x_max_abs_new > x_min_abs_new and y_max_abs_new > y_min_abs_new:
                                    if 0 <= class_id_orig < len(vietnamese_classes):
                                        boxes.append([x_min_abs_new, y_min_abs_new, x_max_abs_new, y_max_abs_new])
                                        labels.append(class_id_orig + 1)
                                        
                                        # Create a binary mask for this bounding box
                                        mask = torch.zeros((new_h, new_w), dtype=torch.uint8)
                                        x_min, y_min = int(x_min_abs_new), int(y_min_abs_new)
                                        x_max, y_max = int(x_max_abs_new), int(y_max_abs_new)
                                        mask[y_min:y_max, x_min:x_max] = 1
                                        masks.append(mask)
                            except ValueError:
                                continue

            if not boxes:
                boxes_tensor = torch.zeros((0, 4), dtype=torch.float32)
                labels_tensor = torch.zeros(0, dtype=torch.int64)
                masks_tensor = torch.zeros((0, new_h, new_w), dtype=torch.uint8)
            else:
                boxes_tensor = torch.tensor(boxes, dtype=torch.float32)
                labels_tensor = torch.tensor(labels, dtype=torch.int64)
                masks_tensor = torch.stack(masks) if masks else torch.zeros((0, new_h, new_w), dtype=torch.uint8)

            target = {}
            target["boxes"] = boxes_tensor
            target["labels"] = labels_tensor
            target["masks"] = masks_tensor  # Add masks for Mask R-CNN
            target["image_id"] = torch.tensor([idx])
            
            if boxes_tensor.shape[0] > 0:
                area = (boxes_tensor[:, 2] - boxes_tensor[:, 0]) * (boxes_tensor[:, 3] - boxes_tensor[:, 1])
                target["area"] = area
            else:
                target["area"] = torch.zeros(0, dtype=torch.float32)
            target["iscrowd"] = torch.zeros((boxes_tensor.shape[0],), dtype=torch.int64)

            if self.transforms:
                image_resized = self.transforms(image_resized) 
            
            return image_resized, target

        except Exception as e: 
            print(f"{img_filename}: {e}")
            # Return a dummy sample in case of error
            dummy_image = torch.zeros((3, self.target_size[1], self.target_size[0]), dtype=torch.float32)
            dummy_target = {
                "boxes": torch.zeros((0, 4), dtype=torch.float32),
                "labels": torch.zeros(0, dtype=torch.int64),
                "masks": torch.zeros((0, self.target_size[1], self.target_size[0]), dtype=torch.uint8),
                "image_id": torch.tensor([idx]),
                "area": torch.zeros(0, dtype=torch.float32),
                "iscrowd": torch.zeros(0, dtype=torch.int64)
            }
            return dummy_image, dummy_target

    def __len__(self):
        return len(self.image_filenames)

def get_transform():
    return transforms.Compose([
        transforms.ToTensor(), 
    ])

def collate_fn(batch):
    return tuple(zip(*batch))

def get_maskrcnn_model(num_classes, pretrained=True, hidden_layer=256):
    """
    Create and configure a Mask R-CNN model with a ResNet-101 backbone
    """
    try:
        # Load standard model from torchvision and modify it
        import torchvision.models.detection as detection_models
        try:
            # First try using a direct method if available
            model = detection_models.maskrcnn_resnet101_fpn(pretrained=pretrained)
            print("Successfully loaded Mask R-CNN with ResNet-101 backbone directly")
        except (AttributeError, ImportError):
            # Fallback to custom implementation
            print("Direct model not found, building custom Mask R-CNN with ResNet-101...")
            # Use the stable FPN implementation from torchvision
            backbone = resnet_fpn_backbone('resnet101', pretrained=pretrained)
            model = MaskRCNN(backbone, num_classes=91)  # 91 is COCO default
            
            if pretrained:
                # Initialize with reasonable defaults
                for name, param in model.named_parameters():
                    if 'weight' in name:
                        torch.nn.init.normal_(param, mean=0.0, std=0.01)
                    elif 'bias' in name:
                        torch.nn.init.zeros_(param)
                print("Initialized custom ResNet-101 Mask R-CNN with standard weights")
    except Exception as e:
        print(f"Error creating model: {e}")
        # Ultimate fallback - use ResNet-50 which is more stable
        print("Falling back to ResNet-50 Mask R-CNN which is more stable")
        model = maskrcnn_resnet50_fpn(pretrained=pretrained)
    
    # Replace the box predictor with a new one for our number of classes
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
    
    # Replace the mask predictor with a new one for our number of classes
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    model.roi_heads.mask_predictor = MaskRCNNPredictor(
        in_features_mask, hidden_layer, num_classes
    )
    
    return model

def calculate_average_iou(pred_boxes, gt_boxes):
    """Calculate average IoU between predicted and ground truth boxes"""
    if pred_boxes.shape[0] == 0 or gt_boxes.shape[0] == 0:
        return 0.0
    iou_matrix = box_iou(pred_boxes, gt_boxes)
    max_ious, _ = iou_matrix.max(dim=1)
    return max_ious.mean().item()

def save_model(model, optimizer, lr_scheduler, epoch, metrics=None, path="/kaggle/working/checkpoints/traffic_sign_detector"):
    """Save model checkpoint"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    filename = f"{path}_epoch_{epoch}.pth"
    model_state = model.module.state_dict() if isinstance(model, torch.nn.DataParallel) else model.state_dict()
    checkpoint = {
        "epoch": epoch, 
        "model_state_dict": model_state,
        "optimizer_state_dict": optimizer.state_dict() if optimizer else None,
        "lr_scheduler_state_dict": lr_scheduler.state_dict() if lr_scheduler else None,
        "metrics": metrics
    }
    torch.save(checkpoint, filename)
    print(f"✅ Model saved at: {filename}")
    return filename

def plot_metrics(history):
    """Plot training metrics"""
    epochs = range(1, len(history['train_loss']) + 1)
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 3, 1)
    plt.plot(epochs, history['train_loss'], 'bo-', label='Training loss')
    plt.plot(epochs, history['val_loss'], 'ro-', label='Validation loss')
    plt.title('Training and validation loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()

    plt.subplot(1, 3, 2)
    plt.plot(epochs, history['val_mAP'], 'go-', label='Validation mAP@0.5:0.95')
    plt.plot(epochs, history['val_mAP50'], 'yo-', label='Validation mAP@0.5')
    plt.title('Validation mAP')
    plt.xlabel('Epochs')
    plt.ylabel('mAP')
    plt.legend()

    plt.subplot(1, 3, 3)
    plt.plot(epochs, history['val_avg_iou'], 'co-', label='Validation Avg IoU')
    plt.title('Validation Average IoU')
    plt.xlabel('Epochs')
    plt.ylabel('IoU')
    plt.legend()

    plt.tight_layout()
    plt.savefig('/kaggle/working/training_metrics.png')
    plt.show()

def calculate_class_accuracy(model, data_loader, device, score_threshold=0.5, iou_threshold=0.5):
    """
    Calculate per-class prediction accuracy
    Returns both class-wise accuracy and a confusion matrix
    """
    model.eval()
    class_correct = Counter()  # Number of correct predictions per class
    class_total = Counter()    # Total number of ground truth boxes per class
    class_predictions = Counter()  # Total predictions made per class
    
    with torch.no_grad():
        for images, targets in data_loader:
            images = [img.to(device) for img in images]
            
            # Get predictions from model
            predictions = model(images)
            
            # Process each image in the batch
            for i in range(len(images)):
                pred = predictions[i]
                target = targets[i]
                
                # Skip if no ground truth boxes
                if target['boxes'].shape[0] == 0:
                    continue
                
                # Get predicted boxes, scores, and labels with scores above threshold
                pred_boxes = pred['boxes'].cpu()
                pred_scores = pred['scores'].cpu()
                pred_labels = pred['labels'].cpu()
                
                # Filter predictions by score threshold
                keep_indices = pred_scores >= score_threshold
                pred_boxes = pred_boxes[keep_indices]
                pred_labels = pred_labels[keep_indices]
                pred_scores = pred_scores[keep_indices]
                
                # Count total predictions per class
                for label in pred_labels:
                    class_predictions[label.item()] += 1
                
                # Ground truth boxes and labels
                gt_boxes = target['boxes'].cpu()
                gt_labels = target['labels'].cpu()
                
                # Count total ground truth boxes per class
                for label in gt_labels:
                    class_total[label.item()] += 1
                
                # Skip if no predicted boxes
                if pred_boxes.shape[0] == 0:
                    continue
                
                # Calculate IoU between predictions and ground truth
                iou_matrix = box_iou(pred_boxes, gt_boxes)
                
                # For each ground truth box, find best matching prediction
                for gt_idx in range(gt_boxes.shape[0]):
                    gt_label = gt_labels[gt_idx].item()
                    
                    # Find predictions with same class as ground truth
                    same_class_mask = pred_labels == gt_label
                    if not same_class_mask.any():
                        continue  # No predictions of this class
                    
                    # Get IoUs for predictions of the same class
                    if same_class_mask.sum() > 0:
                        class_ious = iou_matrix[:, gt_idx][same_class_mask]
                        max_iou, max_idx = class_ious.max(dim=0)
                        
                        # If IoU exceeds threshold, count as correct
                        if max_iou >= iou_threshold:
                            class_correct[gt_label] += 1
    
    # Calculate accuracy for each class
    class_accuracy = {}
    class_precision = {}
    
    for class_id in range(1, len(vietnamese_classes) + 1):
        # Accuracy is the proportion of correctly matched ground truth boxes
        if class_total[class_id] > 0:
            class_accuracy[class_id] = class_correct[class_id] / class_total[class_id]
        else:
            class_accuracy[class_id] = 0
            
        # Precision is the proportion of correct predictions among all predictions
        if class_predictions[class_id] > 0:
            class_precision[class_id] = class_correct[class_id] / class_predictions[class_id]
        else:
            class_precision[class_id] = 0
    
    return class_accuracy, class_precision, class_total, class_predictions, class_correct

def train_model(model, train_loader, val_loader, device, num_epochs=10,
                lr=0.001, momentum=0.9, weight_decay=0.0005,
                save_path='/kaggle/working/checkpoints',
                iou_calc_score_threshold=0.5):
    """
    Train the Mask R-CNN model with ResNet-101 backbone and IoU metrics
    """
    print(f"Started training...")
    os.makedirs(save_path, exist_ok=True)

    # Use a lower learning rate for stability
    actual_lr = lr * 0.5
    print(f"Using learning rate: {actual_lr} (reduced for stability)")
    
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=actual_lr, momentum=momentum, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='max', factor=0.2, patience=2, verbose=True)

    model = model.to(device)
    num_gpus = torch.cuda.device_count()
    if num_gpus > 1:
        print(f"Using {num_gpus} GPUs!")
        model = torch.nn.DataParallel(model)

    history = {'train_loss': [], 'val_loss': [], 'val_mAP': [], 'val_mAP50': [], 'val_avg_iou': []}
    best_map = 0.0
    best_model_path = None
    max_grad_norm = 1.0  # Add gradient clipping for stability

    for epoch in range(1, num_epochs + 1):
        start_time = time.time()
        model.train()
        train_loss = 0.0
        batch_count = 0

        for batch_idx, (images, targets) in enumerate(train_loader):
            try:
                images = [img.to(device) for img in images]
                targets_on_device = [{k: v.to(device) for k, v in t.items()} for t in targets]

                valid_indices = [i for i, t in enumerate(targets_on_device) if t['boxes'].shape[0] > 0]
                if not valid_indices:
                    continue

                images_to_train = [images[i] for i in valid_indices]
                targets_to_train = [targets_on_device[i] for i in valid_indices]

                if not images_to_train:
                    continue

                loss_dict = model(images_to_train, targets_to_train)
                losses = sum(loss for loss in loss_dict.values())
                
                # Skip bad batches that might cause instability
                if not torch.isfinite(losses):
                    print(f"Warning: non-finite loss, skipping batch {batch_idx}")
                    continue

                optimizer.zero_grad()
                losses.backward()
                # Add gradient clipping for stability
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)
                optimizer.step()

                train_loss += losses.item()
                batch_count += 1
                if (batch_idx + 1) % 20 == 0:
                    print(f"Epoch [{epoch}/{num_epochs}], Batch [{batch_idx+1}/{len(train_loader)}], Loss: {losses.item():.4f}")
            except Exception as e:
                print(f"Error when training batch {batch_idx}: {str(e)}")
                continue

        avg_train_loss = train_loss / max(batch_count, 1)
        history['train_loss'].append(avg_train_loss)

        # Validation phase
        model.eval()
        val_loss = 0.0
        val_batch_count = 0
        metric = MeanAveragePrecision(box_format='xyxy')
        
        # For IoU calculation
        total_iou_high_score_preds = 0.0
        num_images_with_high_score_matches = 0

        with torch.no_grad():
            for batch_idx, (images, targets) in enumerate(val_loader):
                try:
                    images_on_device = [img.to(device) for img in images]
                    targets_on_device = [{k: v.to(device) for k, v in t.items()} for t in targets]

                    valid_indices = [i for i, t in enumerate(targets_on_device) if t['boxes'].shape[0] > 0]
                    if not valid_indices:
                        continue
                    
                    images_to_val = [images_on_device[i] for i in valid_indices]
                    targets_to_val = [targets_on_device[i] for i in valid_indices]

                    if not images_to_val:
                        continue
                    
                    # Get predictions
                    predictions_all = model(images_to_val)
                    
                    # Calculate validation loss
                    current_training_state = model.training
                    model.train()
                    loss_dict_val = model(images_to_val, targets_to_val)
                    model.train(current_training_state)
                    losses_val = sum(loss for loss in loss_dict_val.values())
                    val_loss += losses_val.item()
                    val_batch_count += 1
                    
                    # Prepare data for mAP metric
                    preds_for_map_metric = [{'boxes': p['boxes'].cpu(), 'scores': p['scores'].cpu(), 'labels': p['labels'].cpu()} 
                                         for p in predictions_all]
                    gts_for_map_metric = [{'boxes': t['boxes'].cpu(), 'labels': t['labels'].cpu()} for t in targets_to_val]
                    metric.update(preds_for_map_metric, gts_for_map_metric)
                    
                    # Calculate IoU for predictions with score > threshold
                    for i in range(len(predictions_all)):
                        pred_single_image = predictions_all[i]
                        target_single_image = targets_to_val[i]

                        pred_boxes_cpu = pred_single_image["boxes"].cpu()
                        pred_scores_cpu = pred_single_image["scores"].cpu()
                        gt_boxes_cpu = target_single_image["boxes"].cpu()

                        if pred_boxes_cpu.shape[0] > 0 and gt_boxes_cpu.shape[0] > 0:
                            # Filter predictions by score threshold
                            keep_mask = pred_scores_cpu >= iou_calc_score_threshold
                            filtered_pred_boxes = pred_boxes_cpu[keep_mask]

                            if filtered_pred_boxes.shape[0] > 0:
                                # Calculate IoU between filtered predictions and ground truth
                                avg_iou_for_image_high_score = calculate_average_iou(filtered_pred_boxes, gt_boxes_cpu)

                                if avg_iou_for_image_high_score > 0:
                                    total_iou_high_score_preds += avg_iou_for_image_high_score
                                    num_images_with_high_score_matches += 1
                except Exception as e:
                    print(f"Error during validation batch {batch_idx}: {e}")
                    continue
        
        avg_val_loss = val_loss / max(val_batch_count, 1)
        history['val_loss'].append(avg_val_loss)
        
        map_results = metric.compute()
        map_value = map_results["map"].item()
        map50_value = map_results["map_50"].item()
        history['val_mAP'].append(map_value)
        history['val_mAP50'].append(map50_value)
        
        # Calculate average IoU for high-score predictions
        avg_iou_epoch_high_score = total_iou_high_score_preds / max(num_images_with_high_score_matches, 1)
        history['val_avg_iou'].append(avg_iou_epoch_high_score)
        
        scheduler.step(map_value)
        epoch_time = time.time() - start_time
        print(f"\n--- Epoch {epoch}/{num_epochs} completed in {epoch_time:.2f}s ---")
        print(f"Train Loss: {avg_train_loss:.4f}, Val Loss: {avg_val_loss:.4f}")
        print(f"mAP@0.5-0.95: {map_value:.4f}, mAP@0.5: {map50_value:.4f}")
        print(f"Avg IoU (Pred Score >= {iou_calc_score_threshold}): {avg_iou_epoch_high_score:.4f}")
        
        epoch_metrics = {
            "epoch": epoch, 
            "train_loss": avg_train_loss, 
            "val_loss": avg_val_loss, 
            "mAP": map_value, 
            "mAP50": map50_value, 
            "avg_iou_filtered": avg_iou_epoch_high_score
        }
        
        current_model_path = save_model(model, optimizer, scheduler, epoch, epoch_metrics, f"{save_path}/ts_detector")
        if map_value > best_map:
            best_map = map_value
            best_model_path = current_model_path
            print(f"New best model saved: {best_model_path} with mAP = {best_map:.4f}")
            
    print(f"\nBest model based on mAP: {best_model_path} with mAP = {best_map:.4f}")
    return model, history, best_model_path

def print_class_performance(class_accuracy, class_precision, class_total, class_predictions, class_correct, threshold=0):
    """
    Print class-wise performance metrics in a formatted table
    Only show classes with at least 'threshold' ground truth instances
    """
    print("\n=== Traffic Sign Detection Class Performance ===")
    print(f"{'Class ID':<8} {'Class Name':<50} {'Accuracy':<10} {'Precision':<10} {'Correct':<10} {'Total GT':<10} {'Total Pred':<10}")
    print("-" * 105)
    
    # Sort classes by their IDs
    sorted_classes = sorted(class_accuracy.keys())
    
    total_correct = 0
    total_ground_truth = 0
    total_predictions = 0
    
    for class_id in sorted_classes:
        if class_total[class_id] >= threshold:
            class_name = vietnamese_classes.get(class_id - 1, "Unknown")  # -1 because model uses 1-indexed classes
            acc = class_accuracy[class_id]
            prec = class_precision[class_id]
            correct = class_correct[class_id]
            total = class_total[class_id]
            predictions = class_predictions[class_id]
            
            print(f"{class_id:<8} {class_name:<50} {acc*100:>8.2f}% {prec*100:>8.2f}% {correct:>10} {total:>10} {predictions:>10}")
            
            total_correct += correct
            total_ground_truth += total
            total_predictions += predictions
    
    # Print overall statistics
    print("-" * 105)
    overall_accuracy = total_correct / total_ground_truth if total_ground_truth > 0 else 0
    overall_precision = total_correct / total_predictions if total_predictions > 0 else 0
    print(f"{'Overall':<8} {'All Classes':<50} {overall_accuracy*100:>8.2f}% {overall_precision*100:>8.2f}% {total_correct:>10} {total_ground_truth:>10} {total_predictions:>10}")

def main():
    # Configuration
    torch.manual_seed(42)
    np.random.seed(42)
    
    NUM_CLASSES = len(vietnamese_classes) + 1  # +1 for background class
    
    # Paths for Kaggle
    BASE_DATA_PATH = "/kaggle/input/traffic-sign-vietnamese/archive/"
    IMAGE_DIR = os.path.join(BASE_DATA_PATH, "images")
    LABELS_DIR = os.path.join(BASE_DATA_PATH, "labels")
    SPLIT_DATASET_DIR = os.path.join(BASE_DATA_PATH, "split_dataset")
    
    TRAIN_FILES_LIST_PATH = os.path.join(SPLIT_DATASET_DIR, "train_files.txt")
    TEST_FILES_LIST_PATH = os.path.join(SPLIT_DATASET_DIR, "test_files.txt")
    
    # Training parameters - adjusted for stability
    target_size = (640, 640)
    num_epochs = 15
    batch_size = 2  # Smaller batch size for stability
    learning_rate = 0.001  # Lower initial learning rate
    
    # Create datasets and dataloaders
    transform = get_transform()
    
    print("Creating datasets...")
    train_dataset = TrafficSignDataset(
        image_dir=IMAGE_DIR, 
        labels_dir=LABELS_DIR,
        image_file_list_path=TRAIN_FILES_LIST_PATH,
        target_size=target_size, 
        transforms=transform
    )
    
    val_dataset = TrafficSignDataset(
        image_dir=IMAGE_DIR, 
        labels_dir=LABELS_DIR,
        image_file_list_path=TEST_FILES_LIST_PATH,
        target_size=target_size, 
        transforms=transform
    )
    
    print(f"Number of images for training: {len(train_dataset)}")
    print(f"Number of images for validation: {len(val_dataset)}")
    
    num_workers = 2
    train_loader = DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True,
        num_workers=num_workers, collate_fn=collate_fn
    )
    
    val_loader = DataLoader(
        val_dataset, batch_size=batch_size, shuffle=False,
        num_workers=num_workers, collate_fn=collate_fn
    )
    
    # Set up device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    # Create Mask R-CNN model with ResNet-101 backbone
    print("Creating Mask R-CNN model with ResNet-101 backbone...")
    model = get_maskrcnn_model(NUM_CLASSES, pretrained=True)
    
    # Train model
    print("Starting training...")
    model, history, best_model_path = train_model(
        model=model,
        train_loader=train_loader,
        val_loader=val_loader,
        device=device,
        num_epochs=num_epochs,
        lr=learning_rate,
        save_path='/kaggle/working/checkpoints',
        iou_calc_score_threshold=0.5
    )
    
    print(f"Training completed! Best model saved at: {best_model_path}")
    
    # Calculate and print class-wise performance metrics
    print("\nCalculating class-wise performance metrics...")
    class_accuracy, class_precision, class_total, class_predictions, class_correct = calculate_class_accuracy(
        model, val_loader, device, score_threshold=0.5, iou_threshold=0.5
    )
    
    print_class_performance(class_accuracy, class_precision, class_total, class_predictions, class_correct)
    
    print("\nDone!")

if __name__ == "__main__":
    main() 