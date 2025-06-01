import numpy as np
import matplotlib.pyplot as plt
import joblib
from sklearn.metrics import confusion_matrix, classification_report
import seaborn as sns
import random
import tensorflow as tf

def visualize_model_predictions(model, data, labels=None, class_names=None, 
                              is_generator=False, num_images=25, grid_size=(5, 5),
                              figsize=(20, 20)):
    """
    General function to visualize model predictions on test data.
    
    Parameters:
    -----------
    model : trained model object
        A trained model with a predict method (sklearn, tensorflow, etc.)
    data : array-like or generator
        Test data or data generator
    labels : array-like, optional
        True labels (if not using a generator)
    class_names : dict or list, optional
        Class names for each class index
    is_generator : bool, default=False
        Whether the data is a generator (like in Keras)
    num_images : int, default=25
        Number of images to visualize
    grid_size : tuple, default=(5, 5)
        Grid dimensions for the plot
    figsize : tuple, default=(20, 20)
        Figure size
    """
    # Get images and labels based on input type
    if is_generator:
        # Working with a generator (like Keras ImageDataGenerator)
        images, true_labels = [], []
        data.reset()
        samples_needed = num_images
        
        while samples_needed > 0:
            try:
                batch_x, batch_y = next(data)
                images.append(batch_x)
                true_labels.append(batch_y)
                samples_needed -= batch_x.shape[0]
            except StopIteration:
                data.reset()
                continue
                
            if samples_needed <= 0:
                break
                
        images = np.vstack(images)
        true_labels = np.vstack(true_labels)
        
        # Choose random samples
        indices = np.random.choice(images.shape[0], num_images, replace=False)
        selected_images = images[indices]
        true_labels = true_labels[indices]
        
        # Make predictions
        predictions = model.predict(selected_images)
        
        # Convert one-hot encoded labels to class indices
        if len(true_labels.shape) > 1 and true_labels.shape[1] > 1:
            true_classes = np.argmax(true_labels, axis=1)
        else:
            true_classes = true_labels
            
        # Convert predictions to class indices
        if len(predictions.shape) > 1 and predictions.shape[1] > 1:
            pred_classes = np.argmax(predictions, axis=1)
        else:
            pred_classes = predictions.astype(int)
            
        # Get class names
        if class_names is None and hasattr(data, 'class_indices'):
            class_indices_inv = {v: k for k, v in data.class_indices.items()}
            class_names = [class_indices_inv.get(i, str(i)) for i in range(max(class_indices_inv.values()) + 1)]
    else:
        # Working with direct data arrays (like in sklearn)
        images = data
        true_classes = labels
        
        # Choose random samples
        indices = np.random.choice(len(images), num_images, replace=False)
        selected_images = images[indices]
        true_classes = true_classes[indices]
        
        # Make predictions
        pred_classes = model.predict(selected_images)
    
    # Plotting
    rows, cols = grid_size
    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    axes = axes.ravel()
    
    for i in range(min(num_images, len(axes))):
        ax = axes[i]
        img = selected_images[i].copy()
        
        # Normalize image for display if needed
        if img.max() > 1.0:
            img = img / 255.0
            
        # Handle different image formats
        if len(img.shape) == 3:
            if img.shape[2] == 1:  # Grayscale with channel dimension
                img = img.reshape(img.shape[0], img.shape[1])
        elif len(img.shape) == 1:  # Flattened image
            # Try to find appropriate dimensions
            dim = int(np.sqrt(img.shape[0]))
            if dim * dim == img.shape[0]:
                img = img.reshape(dim, dim)
            else:
                # Attempt to reshape based on common image sizes if perfect square not found
                for hw in [(28, 28), (32, 32), (64, 64), (224, 224)]:
                    if hw[0] * hw[1] == img.shape[0]:
                        img = img.reshape(hw)
                        break
        
        # Display image
        if len(img.shape) == 3:
            ax.imshow(img)
        else:
            ax.imshow(img, cmap='gray')
        
        # Get true and predicted class info
        true_class = true_classes[i]
        pred_class = pred_classes[i]
        
        # Get class names if available
        if class_names is not None:
            if isinstance(class_names, dict):
                true_name = class_names.get(true_class, str(true_class))
                pred_name = class_names.get(pred_class, str(pred_class))
            else:  # Assume list or array
                true_name = class_names[true_class] if true_class < len(class_names) else str(true_class)
                pred_name = class_names[pred_class] if pred_class < len(class_names) else str(pred_class)
            title_text = f"Actual: {true_name}\nPredict: {pred_name}"
        else:
            title_text = f"Actual: {true_class}\nPredict: {pred_class}"
        
        # Set color based on correctness
        color = 'green' if true_class == pred_class else 'red'
        ax.set_title(title_text, color=color)
        ax.axis('off')
    
    # Hide any unused subplots
    for i in range(num_images, len(axes)):
        axes[i].axis('off')
        
    plt.tight_layout()
    plt.suptitle("Test Images: Correct vs. Incorrect Predictions", fontsize=16)
    plt.subplots_adjust(top=0.95)
    plt.show()
    
    # Print accuracy statistics
    correct = (pred_classes == true_classes)
    accuracy = correct.mean()
    print(f"Accuracy: {accuracy:.4f}")
    print(f"Correct predictions: {correct.sum()}")
    print(f"Incorrect predictions: {(~correct).sum()}")
    
    return correct, pred_classes, true_classes

# Example usage with a random forest model:
if __name__ == "__main__":
    # Example 1: Using with scikit-learn model and direct data
    try:
        # Load the saved model
        rf_model = joblib.load('random_forest_traffic_sign_model.joblib')
        
        # Load and prepare test data (assuming X_test and y_test exist)
        # X_test = ...
        # y_test = ...
        
        # Visualize predictions
        visualize_model_predictions(rf_model, X_test, y_test, num_images=16, grid_size=(4, 4))
        
        # Visualize only incorrect predictions
        correct, y_pred, y_true = visualize_model_predictions(rf_model, X_test, y_test)
        incorrect_idx = np.where(~correct)[0]
        if len(incorrect_idx) > 0:
            print("\nVisualizing incorrect predictions:")
            visualize_model_predictions(rf_model, X_test[incorrect_idx], y_test[incorrect_idx])
        
    except Exception as e:
        print(f"Example 1 failed: {e}")
    
    # Example 2: Using with a Keras model and data generator
    try:
        # Assuming you have a Keras model and generator
        # model = tf.keras.models.load_model('cnn_model.h5')
        # test_generator = ...
        # class_names = [...]
        
        # Visualize predictions
        # visualize_model_predictions(model, test_generator, is_generator=True, class_names=class_names)
        pass
    except Exception as e:
        print(f"Example 2 failed: {e}") 