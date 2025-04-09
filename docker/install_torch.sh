#!/usr/bin/env bash

# Print usage if no arguments are provided
if [[ $# -eq 0 ]]; then
    echo "Usage: $0 --cuda-version <version>"
    exit 1
fi

# Parse the arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --cuda-version)
            cuda_version="$2"
            shift 2
            ;;
        *)
            echo "Error: Unknown parameter: $1"
            echo "Usage: $0 --cuda-version <version>"
            exit 1
            ;;
    esac
done

if [[ -z "$cuda_version" ]]; then
    echo "Error: --cuda-version requires an argument."
    echo "Usage: $0 --cuda-version <version>"
    exit 1
fi

# Determine installation command based on CUDA version.
case "$cuda_version" in
    cpu)
        install_cmd="pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu"
        ;;
    11.8)
        install_cmd="pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118"
        ;;
    12.4)
        install_cmd="pip install torch torchvision torchaudio"
        ;;
    12.6)
        install_cmd="pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126"
        ;;
    *)
        echo "Unsupported CUDA version: $cuda_version"
        exit 1
        ;;
esac

echo "Installing Torch for CUDA version: $cuda_version"
eval $install_cmd

if [[ $? -eq 0 ]]; then
    echo "Torch installation complete for CUDA version: $cuda_version"
else
    echo "Torch installation failed for CUDA version: $cuda_version"
    exit 1
fi