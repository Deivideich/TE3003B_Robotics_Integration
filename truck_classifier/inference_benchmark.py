import torch
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import torch.nn as nn
from torchvision.models import efficientnet_v2_s
import argparse

# Define hyperparameters
IMAGE_SIZE = 500
NUM_CLASSES = 3
BATCH_SIZE = 8

# Define the model structure
def create_model():
    model = efficientnet_v2_s(pretrained=False)
    model.classifier[1] = nn.Linear(model.classifier[1].in_features, NUM_CLASSES)
    return model

# Define the test function
def test_model(model, test_loader, device):
    model.eval()
    correct = 0
    total = 0

    with torch.no_grad():
        for images, labels in test_loader:
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            _, predicted = outputs.max(1)
            total += labels.size(0)
            correct += predicted.eq(labels).sum().item()

    accuracy = 100. * correct / total
    return accuracy

def main():
    parser = argparse.ArgumentParser(description="Inference Benchmark")
    parser.add_argument('--model', type=str, required=True, help="Path to the model file")
    args = parser.parse_args()

    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Define transforms
    transform = transforms.Compose([
        transforms.Resize((IMAGE_SIZE, IMAGE_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225])
    ])

    # Load test dataset
    test_folder = "test_images"  # Update this path as needed
    test_dataset = datasets.ImageFolder(
        root=test_folder,
        transform=transform
    )

    test_loader = DataLoader(
        test_dataset,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=4
    )

    # Load the model
    model = create_model()
    model.load_state_dict(torch.load(args.model))
    model = model.to(device)

    # Test the model
    accuracy = test_model(model, test_loader, device)
    print(f"Test Accuracy: {accuracy:.2f}%")

if __name__ == "__main__":
    main()