import os
import argparse
import random
import shutil
from pathlib import Path
parser = argparse.ArgumentParser(description="Split dataset into train, validation, and test sets")
parser.add_argument('--input', type=str, required=True, 
                    help="Path to the input folder containing class subfolders")
parser.add_argument('--output', type=str, required=True, 
                    help="Name of the output dataset folder to create")
parser.add_argument('--train-ratio', type=float, default=0.7,
                    help="Ratio of data for training (default: 0.7)")
parser.add_argument('--val-ratio', type=float, default=0.2,
                    help="Ratio of data for validation (default: 0.2)")
parser.add_argument('--test-ratio', type=float, default=0.1,
                    help="Ratio of data for testing (default: 0.1)")
parser.add_argument('--seed', type=int, default=42,
                    help="Random seed for reproducibility (default: 42)")

args = parser.parse_args()
    
def split_dataset(source_folder, output_folder, train_ratio=0.7, val_ratio=0.2, test_ratio=0.1):
    """
    Split dataset into train, validation, and test sets
    """
    # Ensure ratios sum to 1
    assert abs(train_ratio + val_ratio + test_ratio - 1.0) < 1e-6, "Ratios must sum to 1.0"
    
    # Create output directory structure
    splits = ['train', 'val', 'test']
    for split in splits:
        split_path = os.path.join(output_folder, split)
        os.makedirs(split_path, exist_ok=True)
    
    # Get all class folders
    class_folders = [f for f in os.listdir(source_folder) 
                    if os.path.isdir(os.path.join(source_folder, f))]
    
    print(f"Found {len(class_folders)} classes: {class_folders}")
    
    for class_name in class_folders:
        class_path = os.path.join(source_folder, class_name)
        
        # Create class folders in each split
        for split in splits:
            split_class_path = os.path.join(output_folder, split, class_name)
            os.makedirs(split_class_path, exist_ok=True)
        
        # Get all images in this class
        image_files = [f for f in os.listdir(class_path) 
                      if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]
        
        # Shuffle images randomly
        random.shuffle(image_files)
        
        # Calculate split indices
        n_images = len(image_files)
        n_train = int(n_images * train_ratio)
        n_val = int(n_images * val_ratio)
        n_test = n_images - n_train - n_val  # Remaining images go to test
        
        # Split images
        train_images = image_files[:n_train]
        val_images = image_files[n_train:n_train + n_val]
        test_images = image_files[n_train + n_val:]
        
        print(f"Class '{class_name}': {len(train_images)} train, {len(val_images)} val, {len(test_images)} test")
        
        # Copy images to respective folders
        splits_data = [
            ('train', train_images),
            ('val', val_images),
            ('test', test_images)
        ]
        
        for split_name, image_list in splits_data:
            for image_file in image_list:
                src_path = os.path.join(class_path, image_file)
                dst_path = os.path.join(output_folder, split_name, class_name, image_file)
                shutil.copy2(src_path, dst_path)

def main():
    
    
    # Set random seed for reproducibility
    random.seed(args.seed)
    
    # Validate input folder exists
    if not os.path.exists(args.input):
        print(f"Error: Input folder '{args.input}' does not exist!")
        return
    
    # Validate ratios
    total_ratio = args.train_ratio + args.val_ratio + args.test_ratio
    if abs(total_ratio - 1.0) > 1e-6:
        print(f"Error: Ratios must sum to 1.0, got {total_ratio}")
        return
    
    print(f"Splitting dataset from '{args.input}' to '{args.output}'")
    print(f"Ratios - Train: {args.train_ratio}, Val: {args.val_ratio}, Test: {args.test_ratio}")
    print(f"Random seed: {args.seed}")
    
    # Split the dataset
    split_dataset(args.input, args.output, args.train_ratio, args.val_ratio, args.test_ratio)
    
    print(f"\nDataset split completed! Output saved to '{args.output}'")
    print(f"Directory structure:")
    print(f"{args.output}/")
    print(f"├── train/")
    print(f"├── val/")
    print(f"└── test/")

if __name__ == "__main__":
    main()