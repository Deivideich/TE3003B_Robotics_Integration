import torch
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader
import os
from datetime import datetime
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter  # Add this import
import json

import torch.nn as nn
import torch.optim as optim
from torchvision.models import efficientnet_v2_s
import argparse

# Define hyperparameters
BATCH_SIZE = 8
NUM_EPOCHS = 25
LEARNING_RATE = 0.0005
IMAGE_SIZE = 100
NUM_CLASSES = 3

def create_model():
    # Use EfficientNetV2 as backbone
    model = efficientnet_v2_s(pretrained=True)
    # Modify the final layer for our 3 classes
    model.classifier[1] = nn.Linear(model.classifier[1].in_features, NUM_CLASSES)
    return model

def train_model(model, train_loader, criterion, optimizer, device, writer, epoch):  # Add writer and epoch parameters
    model.train()
    running_loss = 0.0
    correct = 0
    total = 0

    for i, (images, labels) in enumerate(tqdm(train_loader)):
        images, labels = images.to(device), labels.to(device)
        
        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item()
        _, predicted = outputs.max(1)
        total += labels.size(0)
        correct += predicted.eq(labels).sum().item()

        # Log batch-wise metrics
        if i % 10 == 0:  # Log every 10 batches
            writer.add_scalar('Training/Batch Loss', loss.item(), epoch * len(train_loader) + i)
    
    epoch_loss = running_loss / len(train_loader)
    epoch_acc = 100. * correct / total
    
    # Log epoch-wise metrics
    writer.add_scalar('Training/Epoch Loss', epoch_loss, epoch)
    writer.add_scalar('Training/Epoch Accuracy', epoch_acc, epoch)
    
    return epoch_loss, epoch_acc

def validate_model(model, val_loader, criterion, device):
    model.eval()
    running_loss = 0.0
    correct = 0
    total = 0

    with torch.no_grad():
        for images, labels in val_loader:
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            loss = criterion(outputs, labels)
            
            running_loss += loss.item()
            _, predicted = outputs.max(1)
            total += labels.size(0)
            correct += predicted.eq(labels).sum().item()
    
    val_loss = running_loss / len(val_loader)
    val_acc = 100. * correct / total
    
    return val_loss, val_acc

def save_label_mapping(label_mapping, model_path):
    label_mapping_path = model_path.replace('.pth', '.json')
    with open(label_mapping_path, 'w') as f:
        json.dump(label_mapping, f)
    print(f"Label mapping saved at {label_mapping_path}")

def main():
    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Initialize TensorBoard writer
    writer_name = f"truck_classifier_experiment_{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    writer = SummaryWriter(f"runs/{writer_name}")

    # Define transforms
    train_transform = transforms.Compose([
        transforms.Resize((IMAGE_SIZE, IMAGE_SIZE)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomRotation(degrees=10),
        transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                           std=[0.229, 0.224, 0.225])
    ])

    val_transform = transforms.Compose([
        transforms.Resize((IMAGE_SIZE, IMAGE_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                           std=[0.229, 0.224, 0.225])
    ])

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-folder', type=str, required=True, help='Path to the dataset folder (should contain train/ and val/ subfolders)')
    args = parser.parse_args()

    # Construct paths for train and validation
    train_folder = os.path.join(args.dataset_folder, 'train')
    val_folder = os.path.join(args.dataset_folder, 'val')

    # Validate that train folder exists
    if not os.path.exists(train_folder):
        print(f"Error: Training folder '{train_folder}' does not exist!")
        return

    # Load training dataset
    train_dataset = datasets.ImageFolder(
        root=train_folder,
        transform=train_transform
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=BATCH_SIZE,
        shuffle=True,
        num_workers=4
    )

    # Load validation dataset if available
    val_loader = None
    if os.path.exists(val_folder):
        val_dataset = datasets.ImageFolder(
            root=val_folder,
            transform=val_transform
        )
        val_loader = DataLoader(
            val_dataset,
            batch_size=BATCH_SIZE,
            shuffle=False,
            num_workers=4
        )
        print(f"Validation dataset loaded with {len(val_dataset)} images")
    else:
        print("No validation folder found. Training without validation.")

    # Create model
    model = create_model()
    model = model.to(device)

    # Add model graph to tensorboard
    sample_images, _ = next(iter(train_loader))
    writer.add_graph(model, sample_images.to(device))

    # Define loss function and optimizer
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    # Training loop with validation
    print("Starting training...")
    best_val_acc = 0.0
    best_model_path = None
    
    for epoch in range(NUM_EPOCHS):
        train_loss, train_acc = train_model(model, train_loader, criterion, optimizer, device, writer, epoch)
        
        # Validate if validation loader is available
        if val_loader:
            val_loss, val_acc = validate_model(model, val_loader, criterion, device)
            
            # Log validation metrics
            writer.add_scalar('Validation/Loss', val_loss, epoch)
            writer.add_scalar('Validation/Accuracy', val_acc, epoch)
            
            print(f'Epoch [{epoch+1}/{NUM_EPOCHS}] Train Loss: {train_loss:.4f} Train Acc: {train_acc:.2f}% | Val Loss: {val_loss:.4f} Val Acc: {val_acc:.2f}%')
            
            # Save best model based on validation accuracy
            if val_acc > best_val_acc:
                best_val_acc = val_acc
                os.makedirs('models', exist_ok=True)
                best_model_path = os.path.join('models', f"truck_classifier_best_{datetime.now().strftime('%Y%m%d-%H%M%S')}.pth")
                torch.save(model.state_dict(), best_model_path)
                print(f"New best model saved with validation accuracy: {val_acc:.2f}%")
        else:
            print(f'Epoch [{epoch+1}/{NUM_EPOCHS}] Train Loss: {train_loss:.4f} Train Acc: {train_acc:.2f}%')

    # Save the final model
    os.makedirs('models', exist_ok=True)
    final_model_path = os.path.join('models', f"truck_classifier_final_{datetime.now().strftime('%Y%m%d-%H%M%S')}.pth")
    torch.save(model.state_dict(), final_model_path)

    # Save label mapping for both best and final models
    label_mapping = {idx: class_name for class_name, idx in train_dataset.class_to_idx.items()}
    save_label_mapping(label_mapping, final_model_path)
    if best_model_path:
        save_label_mapping(label_mapping, best_model_path)

    print(f"Training completed!")
    print(f"Final model saved at: {final_model_path}")
    if best_model_path:
        print(f"Best model saved at: {best_model_path} (Val Acc: {best_val_acc:.2f}%)")
    
    # Close tensorboard writer
    writer.close()

if __name__ == "__main__":
    main()