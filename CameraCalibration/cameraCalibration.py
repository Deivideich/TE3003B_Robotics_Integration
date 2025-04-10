### CALIBRATE CAMERA ###
# Import necessary libraries
import cv2
import numpy as np
import glob
import os


# Define the dimensions of the chessboard
# chessboard_size = (9, 6)  # Number of inner corners per a chessboard row and column
# square_size = 0.022  # Size of a square in meters (25mm)
# # Prepare object points based on the chessboard size
# objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
# objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size
# # Arrays to store object points and image points from all the images
# objpoints = []  # 3d points in real world space
# imgpoints = []  # 2d points in image plane
# # Load images from the specified directory
# images_dir = './images'
# images_path = os.path.join(images_dir, '*.jpeg')
# print(images_path)
# images = glob.glob(images_path)
# print(len(images))
# # Loop through each image and find chessboard corners
# for image_path in images:
#     img = cv2.imread(image_path)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     # Find the chessboard corners
#     ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
#     # If found, add object points and image points
#     if ret:
#         objpoints.append(objp)
#         imgpoints.append(corners)
#         # Draw and display the corners
#         cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
#         cv2.imshow('Chessboard', img)
#         cv2.waitKey(500)
#     else:
#         print(f"Chessboard corners not found in {image_path}")
# # Close all OpenCV windows
# cv2.destroyAllWindows()
# # Perform camera calibration
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# # Save the calibration results
# calibration_data = {
#     'camera_matrix': mtx,
#     'distortion_coefficients': dist,
#     'rotation_vectors': rvecs,
#     'translation_vectors': tvecs
# }
# # Save the calibration data to a file
# calibration_file = './calibration_data.npz'
# np.savez(calibration_file, **calibration_data)
# # Load the calibration data
# calibration_data = np.load(calibration_file)
# camera_matrix = calibration_data['camera_matrix']
# distortion_coefficients = calibration_data['distortion_coefficients']
# # Print the calibration results
# print("Camera Matrix:")
# print(camera_matrix)
# print("Distortion Coefficients:")
# print(distortion_coefficients)


# Load the images to be stitched
images = []
images_dir = './panorama_images'
images_path = os.path.join(images_dir, '*.jpeg')
images = glob.glob(images_path)
print(len(images))
# Read the images
img1 = cv2.imread(images[0])
img2 = cv2.imread(images[1])
img3 = cv2.imread(images[2])
# Resize images to the same size
# img1 = cv2.resize(img1, (640, 480))
# img2 = cv2.resize(img2, (640, 480))
# img3 = cv2.resize(img3, (640, 480))
# Convert images to grayscale
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
gray3 = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)
# Detect ORB keypoints and descriptors
#orb = cv2.SIFT_create()
orb = cv2.ORB_create()
keypoints1, descriptors1 = orb.detectAndCompute(gray1, None)
keypoints2, descriptors2 = orb.detectAndCompute(gray2, None)
keypoints3, descriptors3 = orb.detectAndCompute(gray3, None)

#Draw keypoints on the images
img1_keypoints = cv2.drawKeypoints(img1, keypoints1, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
img2_keypoints = cv2.drawKeypoints(img2, keypoints2, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
img3_keypoints = cv2.drawKeypoints(img3, keypoints3, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# Display the keypoints
cv2.imshow('Image 1 Keypoints', img1_keypoints)
cv2.imshow('Image 2 Keypoints', img2_keypoints)
cv2.imshow('Image 3 Keypoints', img3_keypoints)
cv2.waitKey(0)

# Create a BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#bf = cv2.BFMatcher()
# Match descriptors between the first two images
matches1 = bf.match(descriptors1, descriptors2)
# Sort matches based on distance
matches1 = sorted(matches1, key=lambda x: x.distance)
# Draw matches
img_matches1 = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches1, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
# Display the matches
cv2.imshow('Matches between Image 1 and Image 2', img_matches1)
cv2.waitKey(0)
# Find homography between the first two images
src_pts1 = np.float32([keypoints1[m.queryIdx].pt for m in matches1]).reshape(-1, 1, 2)
dst_pts1 = np.float32([keypoints2[m.trainIdx].pt for m in matches1]).reshape(-1, 1, 2)
M1, mask1 = cv2.findHomography(src_pts1, dst_pts1, cv2.RANSAC, 5.0)
# Apply homography to the first image
h1, w1 = img1.shape[:2]
pts1 = np.float32([[0, 0], [0, h1], [w1, h1], [w1, 0]]).reshape(-1, 1, 2)
dst1 = cv2.perspectiveTransform(pts1, M1)

def blendImages(img1, img2):
    # Create a mask for the second image
    mask = np.zeros_like(img2, dtype=np.uint8)
    mask[img2 > 0] = 255
    # Blend the images using bitwise operations
    blended = cv2.bitwise_and(img1, img1, mask=mask)
    blended += img2
    return blended

def warpImages(img1, img2, H):
    rows1, cols1 = img1.shape[:2]
    rows2, cols2 = img2.shape[:2]

    X_resultant_kernel = cv2.getGaussianKernel(cols1,200)
    Y_resultant_kernel = cv2.getGaussianKernel(rows1,200)
    
    #generating resultant_kernel matrix 
    resultant_kernel = Y_resultant_kernel * X_resultant_kernel.T
    
    #creating mask and normalising by using np.linalg
    # function
    mask_img1 = 255 * resultant_kernel / np.linalg.norm(resultant_kernel)
    
    X_resultant_kernel = cv2.getGaussianKernel(cols2,200)
    Y_resultant_kernel = cv2.getGaussianKernel(rows2,200)
    
    #generating resultant_kernel matrix 
    resultant_kernel = Y_resultant_kernel * X_resultant_kernel.T

    #creating mask and normalising by using np.linalg
    # function
    mask_img2 = 255 * resultant_kernel / np.linalg.norm(resultant_kernel)

    cv2.imshow('Mask Image 1', mask_img1)
    cv2.imshow('Mask Image 2', mask_img2)
    cv2.waitKey(0)

    list_of_points_1 = np.float32([[0,0], [0, rows1],[cols1, rows1], [cols1, 0]]).reshape(-1, 1, 2)
    temp_points = np.float32([[0,0], [0,rows2], [cols2,rows2], [cols2,0]]).reshape(-1,1,2)

    # When we have established a homography we need to warp perspective
    # Change field of view
    list_of_points_2 = cv2.perspectiveTransform(temp_points, H)

    list_of_points = np.concatenate((list_of_points_1,list_of_points_2), axis=0)

    [x_min, y_min] = np.int32(list_of_points.min(axis=0).ravel() - 0.5)
    [x_max, y_max] = np.int32(list_of_points.max(axis=0).ravel() + 0.5)

    translation_dist = [-x_min,-y_min]

    H_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])

    resized_image_2 = cv2.warpPerspective(img2, H_translation.dot(H), (x_max-x_min, y_max-y_min))

    output_mask_2 = cv2.warpPerspective(mask_img2, H_translation.dot(H), (x_max-x_min, y_max-y_min))
    output_mask_1 = np.zeros_like(output_mask_2)
    output_mask_1[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] = mask_img1
    
    alpha_1 = output_mask_1 / (output_mask_2 + output_mask_1)
    alpha_2 = output_mask_2 / (output_mask_2 + output_mask_1)

    # Resize the first image to match the size of the output image
    resized_image_1 = np.zeros_like(resized_image_2)
    resized_image_1[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] = img1
    for i in range(3):
        resized_image_1[:, :, i] = resized_image_1[:, :, i] * alpha_1
    
    for i in range(3):
        resized_image_2[:, :, i] = resized_image_2[:, :, i] * alpha_2

    # # Create a mask for the first image
    # mask_img2 = np.zeros_like(output_img, dtype=np.uint8)
    # mask_img2[output_img > 0] = 255
    # mask_img1 = np.zeros_like(output_img, dtype=np.uint8)
    # mask_img1[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] = 255
    

    cv2.imshow('Mask Image 1', output_mask_1)
    cv2.imshow('Mask Image 2', output_mask_2)
    cv2.imshow('Blended masks_1', alpha_1)
    cv2.imshow('Blended masks_2', alpha_2)
    cv2.imshow('Resized Image 1', resized_image_1)
    cv2.imshow('Resized Image 2', resized_image_2)
    # cv2.imshow("result_img1", result_img1)
    cv2.waitKey(0)

    #ROI = cv2.bitwise_and(mask_img2, mask_img1)    
    

    resized_image_2[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] =resized_image_2[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] + resized_image_1[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] 
    
    return resized_image_2

# Warp the images using the homography matrix
panorama = warpImages(img2, img1, M1)
# panorama = warpImages(img3, panorama, M2)
# Display the panorama
cv2.imshow('Panorama', panorama)
cv2.waitKey(0)



panorama_gray = cv2.cvtColor(panorama, cv2.COLOR_BGR2GRAY)
# Detect ORB keypoints and descriptors in the panorama
keypoints4, descriptors4 = orb.detectAndCompute(panorama_gray, None)

# Match descriptors between the panorama and the third image
matches2 = bf.match(descriptors4, descriptors3)
# Sort matches based on distance
matches2 = sorted(matches2, key=lambda x: x.distance)
# Draw matches
img_matches2 = cv2.drawMatches(panorama, keypoints4, img3, keypoints3, matches2[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
# Display the matches
cv2.imshow('Matches between Panorama and Image 3', img_matches2)
cv2.waitKey(0)
# Find homography between the panorama and the third image
src_pts2 = np.float32([keypoints4[m.queryIdx].pt for m in matches2]).reshape(-1, 1, 2)
dst_pts2 = np.float32([keypoints3[m.trainIdx].pt for m in matches2]).reshape(-1, 1, 2)
M2, mask2 = cv2.findHomography(src_pts2, dst_pts2, cv2.RANSAC, 5.0)
# Apply homography to the panorama
h2, w2 = panorama.shape[:2]
pts2 = np.float32([[0, 0], [0, h2], [w2, h2], [w2, 0]]).reshape(-1, 1, 2)
dst2 = cv2.perspectiveTransform(pts2, M2)
# Warp the panorama using the homography matrix
panorama = warpImages(img3, panorama, M2)
# Display the final panorama
cv2.imshow('Final Panorama', panorama)
cv2.waitKey(0)

# Save the final panorama
panorama_file = './panorama.jpg'
cv2.imwrite(panorama_file, panorama)
# Close all OpenCV windows
cv2.destroyAllWindows()


