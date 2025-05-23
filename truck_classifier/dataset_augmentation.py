import os
import random
from PIL import Image, ImageEnhance, ImageOps, ImageDraw, ExifTags
import numpy as np
from torchvision import transforms
import tqdm
import cv2

# Constants for augmentations
ZOOM_FACTOR = 1.2  # Maximum zoom factor
BRIGHTNESS_FACTOR = 0.5  # Brightness adjustment range (0.5 to 1.5)
CONTRAST_FACTOR = 0.5  # Contrast adjustment range (0.5 to 1.5)
SATURATION_FACTOR = 0.5  # Saturation adjustment range (0.5 to 1.5)
BLOB_COUNT_MIN = 0  # Minimum number of blobs to add
BLOB_COUNT_MAX = 3  # Maximum number of blobs to add
BLOB_SIZE = (5, 15)  # Size range of blobs (min, max)

MULTIPLIER = 10

# load image ensuring rotation is correct
def load_image(image_path):
    try :
        image=Image.open(image_path)
        for orientation in ExifTags.TAGS.keys() : 
            if ExifTags.TAGS[orientation]=='Orientation' : break 
        exif=dict(image._getexif().items())

        if   exif[orientation] == 3 : 
            image=image.rotate(180, expand=True)
        elif exif[orientation] == 6 : 
            image=image.rotate(270, expand=True)
        elif exif[orientation] == 8 : 
            image=image.rotate(90, expand=True)
        return image
    except:
        raise ValueError("Image not found or not readable")

def zoom_image(image, zoom_factor = -1):
    width, height = image.size
    factor = ZOOM_FACTOR if zoom_factor == -1 else zoom_factor
    
    # Calculate new dimensions
    new_width = int(width * factor)
    new_height = int(height * factor)
    
    # Calculate crop box
    left = (new_width - width) // 2
    top = (new_height - height) // 2
    right = left + width
    bottom = top + height
    
    # Resize and crop
    resized = image.resize((new_width, new_height), Image.LANCZOS)
    
    return resized.crop((left, top, right, bottom))

def adjust_brightness(image):
    enhancer = ImageEnhance.Brightness(image)
    factor = random.uniform(1 - BRIGHTNESS_FACTOR, 1 + BRIGHTNESS_FACTOR)
    return enhancer.enhance(factor)

def adjust_contrast(image):
    enhancer = ImageEnhance.Contrast(image)
    factor = random.uniform(1 - CONTRAST_FACTOR, 1 + CONTRAST_FACTOR)
    return enhancer.enhance(factor)

def adjust_saturation(image):
    enhancer = ImageEnhance.Color(image)
    factor = random.uniform(1 - SATURATION_FACTOR, 1 + SATURATION_FACTOR)
    return enhancer.enhance(factor)

def add_blobs(image):
    draw = ImageDraw.Draw(image)
    blob_count = random.randint(BLOB_COUNT_MIN, BLOB_COUNT_MAX)
    for _ in range(blob_count):
        x = random.randint(0, image.width)
        y = random.randint(0, image.height)
        size = random.randint(*BLOB_SIZE)
        draw.ellipse((x, y, x + size, y + size), fill=(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
    return image

# Main augmentation function
def augment_image(image):
    augmentations = [zoom_image, adjust_brightness, adjust_contrast, adjust_saturation, add_blobs]
    random.shuffle(augmentations)
    for aug in augmentations:
        image = aug(image)
        aug_name = aug.__name__
    return image

def main():
    raw_images_folder = "raw_images"  # Folder containing raw images
    augmented_images_folder = "augmented_images"  # Folder to save augmented images
    multiplier = MULTIPLIER  # Multiplier for augmentation

    os.makedirs(augmented_images_folder, exist_ok=True)

    
    for class_folder in os.listdir(raw_images_folder):
        class_path = os.path.join(raw_images_folder, class_folder)
        if not os.path.isdir(class_path):
            continue

        augmented_class_path = os.path.join(augmented_images_folder, class_folder)
        os.makedirs(augmented_class_path, exist_ok=True)

        images = [f for f in os.listdir(class_path) if f.endswith(('.png', '.jpg', '.jpeg'))]
        progress_bar = tqdm.tqdm(total=len(images) * multiplier, desc=f"Augmenting {class_folder}")
        for image_name in images:
            
            image_path = os.path.join(class_path, image_name)
            image = load_image(image_path)
            for i in range(multiplier):
                augmented_image = augment_image(image.copy())
                augmented_image_name = f"{os.path.splitext(image_name)[0]}_aug_{i}.jpg"
                augmented_image.save(os.path.join(augmented_class_path, augmented_image_name))
                progress_bar.update(1)
        progress_bar.close()
    print(f"Augmentation completed. Augmented images saved in '{augmented_images_folder}' folder.")
if __name__ == "__main__":
    main()