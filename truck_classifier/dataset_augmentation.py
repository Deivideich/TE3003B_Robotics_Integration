import os
import random
from PIL import Image, ImageEnhance, ImageOps, ImageDraw, ExifTags, ImageFilter
import numpy as np
from torchvision import transforms
import tqdm
import cv2
import math
import argparse
# Constants for augmentations
ZOOM_FACTOR = 1.2  # Maximum zoom factor
BRIGHTNESS_FACTOR = 0.3  # Brightness adjustment range (0.5 to 1.5)
CONTRAST_FACTOR = 0.2  # Contrast adjustment range (0.5 to 1.5)
SATURATION_FACTOR = 0.3  # Saturation adjustment range (0.5 to 1.5)
HUE_FACTOR = 0.05  # Hue adjustment range (-0.5 to 0.5)
BLUR_FACTOR = 0.25  # Max Blur adjustment range (0 to blur_factor)
NOISE_FACTOR = 0.15  # Noise adjustment range (0 to noise_factor)
BLOB_COUNT_MIN = 0  # Minimum number of blobs to add
BLOB_COUNT_MAX = 2  # Maximum number of blobs to add
BLOB_SIZE = (15, 150)  # Size range of blobs (min, max)
BLOB_PROBABILITY = 0.15  # Probability of adding a blob

MULTIPLIER = 5

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
        return Image.open(image_path)

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

def adjust_hue(image):
    enhancer = ImageEnhance.Color(image)
    factor = random.uniform(1 - HUE_FACTOR, 1 + HUE_FACTOR)
    # Convert to HSV, adjust hue, and convert back to RGB
    hsv_image = image.convert('HSV')
    h, s, v = hsv_image.split()
    h = h.point(lambda p: (p + int(factor * 255)) % 256)
    hsv_image = Image.merge('HSV', (h, s, v))
    return hsv_image.convert('RGB')

def add_blur(image):
    blur_factor = random.uniform(0, BLUR_FACTOR)
    if blur_factor > 0:
        return image.filter(ImageFilter.GaussianBlur(radius=blur_factor))
    return image

def add_noise(image):
    noise_factor = random.uniform(0, NOISE_FACTOR)
    if noise_factor > 0:
        noise = np.random.normal(0, noise_factor, [image.size[1], image.size[0], 3])
        noise = (noise * 255).astype(np.float32)
        noisy_image = np.array(image, dtype=np.float32)
        noisy_image = np.clip(noisy_image + noise, 0, 255).astype(np.uint8)
        return Image.fromarray(noisy_image)
    return image

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
    if random.random() > BLOB_PROBABILITY:
        return image
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
    augmentations = [zoom_image, adjust_brightness, adjust_contrast, adjust_saturation, add_blobs, adjust_hue, add_blur, add_noise]
    random.shuffle(augmentations)
    for aug in augmentations:
        image = aug(image)
        aug_name = aug.__name__
    return image

def main():
    parser = argparse.ArgumentParser(description='Image augmentation script')
    parser.add_argument('--raw', type=str, default="raw_images",
                        help='Folder containing raw images')
    parser.add_argument('--aug', type=str, default="augmented_images",
                        help='Folder to save augmented images')
    args = parser.parse_args()

    raw_images_folder = args.raw
    augmented_images_folder = args.aug
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