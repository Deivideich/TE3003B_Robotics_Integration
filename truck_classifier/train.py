import torch
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader
import os
from datetime import datetime
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter  # Add this import

import torch.nn as nn
import torch.optim as optim
from torchvision.models import efficientnet_v2_s
import argparse

# Define hyperparameters
BATCH_SIZE = 8
NUM_EPOCHS = 20
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

def main():
    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Initialize TensorBoard writer
    writer_name = f"truck_classifier_experiment_{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    writer = SummaryWriter(f"runs/{writer_name}")

    # Define transforms
    transform = transforms.Compose([
        transforms.Resize((IMAGE_SIZE, IMAGE_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                           std=[0.229, 0.224, 0.225])
    ])

    # Load dataset
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--train-folder', type=str, required=True, help='Path to training data folder')
    args = parser.parse_args()

    train_dataset = datasets.ImageFolder(
        root=args.train_folder,
        transform=transform
    )
    
    train_loader = DataLoader(
        train_dataset,
        batch_size=BATCH_SIZE,
        shuffle=True,
        num_workers=4
    )

    # Create model
    model = create_model()
    model = model.to(device)

    # Add model graph to tensorboard
    sample_images, _ = next(iter(train_loader))
    writer.add_graph(model, sample_images.to(device))

    # Define loss function and optimizer
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    # Training loop
    print("Starting training...")
    for epoch in range(NUM_EPOCHS):
        train_loss, train_acc = train_model(model, train_loader, criterion, optimizer, device, writer, epoch)
        print(f'Epoch [{epoch+1}/{NUM_EPOCHS}] Loss: {train_loss:.4f} Acc: {train_acc:.2f}%')

    # Save the model with datetime
    os.makedirs('models', exist_ok=True)
    model_path = f"models/truck_classifier_{datetime.now().strftime('%Y%m%d-%H%M%S')}.pth"
    torch.save(model.state_dict(), model_path)
    print(f"Training completed and model saved at {model_path}!")
    
    # Close tensorboard writer
    writer.close()

if __name__ == "__main__":
    main()