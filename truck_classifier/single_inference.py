import torch
from torchvision import transforms
from PIL import Image
import torch.nn as nn
from torchvision.models import efficientnet_v2_s
import sys
import argparse
import json
import os

# Define hyperparameters
IMAGE_SIZE = 100
NUM_CLASSES = 3

# Define the model structure
def create_model():
    model = efficientnet_v2_s(pretrained=False)
    model.classifier[1] = nn.Linear(model.classifier[1].in_features, NUM_CLASSES)
    return model

# Define the inference function
def single_inference(model, image_path, device):
    # Define transforms
    transform = transforms.Compose([
        transforms.Resize((IMAGE_SIZE, IMAGE_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225])
    ])

    # Load and preprocess the image
    image = Image.open(image_path).convert('RGB')
    image = transform(image).unsqueeze(0).to(device)

    # Perform inference
    model.eval()
    with torch.no_grad():
        outputs = model(image)
        _, predicted = outputs.max(1)

    return predicted.item()

def load_label_mapping(model_path):
    json_path = model_path.replace('.pth', '.json')
    if os.path.exists(json_path):
        with open(json_path, 'r') as f:
            return json.load(f)
    return None

def main():
    parser = argparse.ArgumentParser(description="Single Image Inference")
    parser.add_argument('--model', type=str, required=True, help="Path to the model file")
    parser.add_argument('--image', type=str, required=True, help="Path to the image file")
    args = parser.parse_args()

    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Load the model
    model = create_model()
    model.load_state_dict(torch.load(args.model))
    model = model.to(device)

    # Load label mapping if available
    label_mapping = load_label_mapping(args.model)

    # Perform single inference
    label_id = single_inference(model, args.image, device)
    if label_mapping:
        label_name = label_mapping.get(str(label_id), "Unknown")
        print(f"Predicted Label: {label_id} ({label_name})")
    else:
        print(f"Predicted Label: {label_id}")

if __name__ == "__main__":
    main()