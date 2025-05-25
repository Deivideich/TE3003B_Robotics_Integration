import torch
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import torch.nn as nn
from torchvision.models import efficientnet_v2_s
import argparse
import json
import os
from tqdm import tqdm
parser = argparse.ArgumentParser(description="Inference Benchmark")
parser.add_argument('--model', type=str, required=True, help="Path to the model file")
parser.add_argument('--test-folder', type=str, required=True, help="Path to the test dataset folder")
args = parser.parse_args()

# Define hyperparameters
IMAGE_SIZE = 100
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
        progress_bar = tqdm(total=len(test_loader), desc="Testing")
        for images, labels in test_loader:
            progress_bar.update(1)
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            _, predicted = outputs.max(1)
            total += labels.size(0)
            correct += predicted.eq(labels).sum().item()
        progress_bar.close()
    accuracy = 100. * correct / total
    return accuracy

def calculate_precision_per_label(model, test_loader, device, label_mapping):
    model.eval()
    label_correct = {}
    label_total = {}

    with torch.no_grad():
        for images, labels in test_loader:
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            _, predicted = outputs.max(1)

            for label, prediction in zip(labels, predicted):
                label = label.item()
                prediction = prediction.item()

                if label not in label_total:
                    label_total[label] = 0
                    label_correct[label] = 0

                label_total[label] += 1
                if label == prediction:
                    label_correct[label] += 1

    precision_per_label = {}
    for label in label_total:
        precision_per_label[label] = 100.0 * label_correct[label] / label_total[label]

    return precision_per_label

def load_label_mapping(model_path):
    json_path = model_path.replace('.pth', '.json')
    if os.path.exists(json_path):
        with open(json_path, 'r') as f:
            return json.load(f)
    return None

def main():
    

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
    test_folder = args.test_folder
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

    # Load label mapping if available
    label_mapping = load_label_mapping(args.model)

    # Test the model
    accuracy = test_model(model, test_loader, device)
    print(f"Test Accuracy: {accuracy:.2f}%")

    # Calculate precision per label
    precision_per_label = calculate_precision_per_label(model, test_loader, device, label_mapping)

    # Display precision for each label
    for label_id, precision in precision_per_label.items():
        if label_mapping:
            label_name = label_mapping.get(str(label_id), "Unknown")
            print(f"Label {label_id} ({label_name}): Precision: {precision:.2f}%")
        else:
            print(f"Label {label_id}: Precision: {precision:.2f}%")

if __name__ == "__main__":
    main()