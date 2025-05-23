import os
import random
from PIL import Image, ImageEnhance, ImageOps, ImageDraw, ExifTags
import numpy as np
from torchvision import transforms
import tqdm
import cv2
import math
# Constants for augmentations
ZOOM_FACTOR = 1.2  # Maximum zoom factor
BRIGHTNESS_FACTOR = 0.2  # Brightness adjustment range (0.5 to 1.5)
CONTRAST_FACTOR = 0.1  # Contrast adjustment range (0.5 to 1.5)
SATURATION_FACTOR = 0.25  # Saturation adjustment range (0.5 to 1.5)
BLOB_COUNT_MIN = 0  # Minimum number of blobs to add
BLOB_COUNT_MAX = 3  # Maximum number of blobs to add
BLOB_SIZE = (15, 150)  # Size range of blobs (min, max)

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
    
    factor = random.uniform(1, factor)  # Random zoom factor between 1 and ZOOM_FACTOR
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

def generate_blob_points(center_x, center_y, radius, irregularity=0.5, spikiness=0.5, num_points=12):
    """
    Generate a blob-like shape using a star/polygon algorithm with randomness.
    """
    points = []
    angle_step = 2 * math.pi / num_points

    for i in range(num_points):
        angle = i * angle_step
        # Vary radius for spikiness
        rand_radius = radius * (1 + random.uniform(-spikiness, spikiness))
        # Add offset for irregularity
        offset_angle = angle + random.uniform(-irregularity, irregularity) * angle_step
        x = center_x + rand_radius * math.cos(offset_angle)
        y = center_y + rand_radius * math.sin(offset_angle)
        points.append((x, y))

    return points

def add_blobs(image):
    draw = ImageDraw.Draw(image)
    blob_count = random.randint(BLOB_COUNT_MIN, BLOB_COUNT_MAX)

    for _ in range(blob_count):
        x = random.randint(0, image.width)
        y = random.randint(0, image.height)
        size = random.randint(*BLOB_SIZE)
        color = (
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(100, 255)  # Optional alpha
        )
        blob_points = generate_blob_points(x, y, size, irregularity=0.4, spikiness=0.6, num_points=random.randint(8, 16))
        draw.polygon(blob_points, fill=color)

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