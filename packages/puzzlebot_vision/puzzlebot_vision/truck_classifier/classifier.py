import torch
from torchvision import transforms
from PIL import Image
import torch.nn as nn
from torchvision.models import efficientnet_v2_s
import cv2
import numpy as np
import os
import json
import time
from typing import Optional, Tuple
from numpy.typing import NDArray

class TruckClassifier:
    def __init__(self, model_path):
        self.model_path = model_path
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Create model and load weights
        self.model = self.create_model()
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        
        # Load label mapping
        self.label_mapping = self.load_label_mapping(model_path)
        
        # Define transforms
        self.transform = transforms.Compose([
            transforms.Resize((100, 100)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])
        
        self.warmup()
        
    def warmup(self):
        start_time = time.time()
        # Perform a warmup pass to initialize the model
        dummy_input = torch.randn(1, 3, 100, 100).to(self.device)
        self.model(dummy_input)
        print(f"Warmup completed in {time.time() - start_time:.2f} seconds")
        
    def create_model(self):
        model = efficientnet_v2_s(pretrained=False)
        model.classifier[1] = nn.Linear(model.classifier[1].in_features, 3)
        return model
    
    def load_label_mapping(self, model_path):
        json_path = model_path.replace('.pth', '.json')
        if os.path.exists(json_path):
            with open(json_path, 'r') as f:
                return json.load(f)
        return None
    
    def inference(self, image : NDArray) -> tuple[int, str | None]:
        # Convert image to PIL Image
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb_image)
        
        # Apply transforms
        image_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)
        
        # Perform inference
        self.model.eval()
        with torch.no_grad():
            outputs = self.model(image_tensor)
            _, predicted = outputs.max(1)
        
        label_id = predicted.item()
        if self.label_mapping:
            label_name = self.label_mapping.get(str(label_id), "Unknown")
            return label_id, label_name
        else:
            return label_id, None