### CALIBRATE CAMERA ###
# Import necessary libraries
import cv2
import numpy as np
import glob
import os
import pathlib

DEBUG = True

def create_rectangular_kernel(width, height, rect_number):

    img = np.zeros((height, width))

    for i in range(rect_number):
        # Create a rectangular gradient
        rec_height = height - i * (height // rect_number)
        rec_width = width - i * (width // rect_number)
        rec_intensity = 255 * (i + 1) // rect_number
        # draw at center
        start_x = (width - rec_width) // 2
        start_y = (height - rec_height) // 2
        end_x = start_x + rec_width
        end_y = start_y + rec_height
        cv2.rectangle(img, (start_x, start_y), (end_x, end_y), int(rec_intensity), -1)
    return img

def warpImages(img1, img2, H):
        rows1, cols1 = img1.shape[:2]
        rows2, cols2 = img2.shape[:2]

        # X_resultant_kernel = cv2.getGaussianKernel(cols1,150)
        # Y_resultant_kernel = cv2.getGaussianKernel(rows1,150)
        
        # #generating resultant_kernel matrix 
        # resultant_kernel = Y_resultant_kernel * X_resultant_kernel.T
        
        # #creating mask and normalising by using np.linalg
        # # function
        # mask_img1 = 255 * resultant_kernel / np.linalg.norm(resultant_kernel)
        
        # X_resultant_kernel = cv2.getGaussianKernel(cols2,200)
        # Y_resultant_kernel = cv2.getGaussianKernel(rows2,200)
        
        # #generating resultant_kernel matrix 
        # resultant_kernel = Y_resultant_kernel * X_resultant_kernel.T
        

        # # creating mask and normalising by using np.linalg
        # # function
        # mask_img2 = 255 * resultant_kernel / np.linalg.norm(resultant_kernel)
        
        mask_img1 = create_rectangular_kernel(cols1, rows1, 256)
        mask_img2 = create_rectangular_kernel(cols2, rows2, 256)
        
        list_of_points_1 = np.float32([[0,0], [0, rows1],[cols1, rows1], [cols1, 0]]).reshape(-1, 1, 2)
        temp_points = np.float32([[0,0], [0,rows2], [cols2,rows2], [cols2,0]]).reshape(-1,1,2)

        # When we have established a homography we need to warp perspective
        # Change field of view
        list_of_points_2 = cv2.perspectiveTransform(temp_points, H)

        list_of_points = np.concatenate((list_of_points_1,list_of_points_2), axis=0)

        [x_min, y_min] = np.int32(list_of_points.min(axis=0).ravel() - 0.5)
        [x_max, y_max] = np.int32(list_of_points.max(axis=0).ravel() + 0.5)

        # translation to move the image to the right place
        translation_dist = [-x_min,-y_min]

        H_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])

        # warping image to stitch
        resized_image_2 = cv2.warpPerspective(img2, H_translation.dot(H), (x_max-x_min, y_max-y_min))
        output_mask_2 = cv2.warpPerspective(mask_img2, H_translation.dot(H), (x_max-x_min, y_max-y_min))
        
        # translating image to be stitched
        resized_image_1 = np.zeros_like(resized_image_2)
        resized_image_1[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] = img1
        
        output_mask_1 = np.zeros_like(output_mask_2)
        output_mask_1[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] = mask_img1
        output_mask_1 = np.array(output_mask_1)
        
        # content-aware mask, so only the non-zero pixels are blended
        resized_image_1_gray = cv2.cvtColor(resized_image_1, cv2.COLOR_BGR2GRAY)
        _, filter_mask_1 = cv2.threshold(resized_image_1_gray, 1, 255, cv2.THRESH_BINARY)
        output_mask_1 = cv2.bitwise_and(output_mask_1, output_mask_1, mask=filter_mask_1)
        # erotion is used to prevent the mask from covering black pixels
        kernel = np.ones((2,2), np.uint8)
        output_mask_1 = cv2.erode(output_mask_1, kernel, iterations=1)
        
        resized_image_2_gray = cv2.cvtColor(resized_image_2, cv2.COLOR_BGR2GRAY)
        _, filter_mask_2 = cv2.threshold(resized_image_2_gray, 1, 255, cv2.THRESH_BINARY)
        output_mask_2 = cv2.bitwise_and(output_mask_2, output_mask_2, mask=filter_mask_2)
        output_mask_2 = cv2.erode(output_mask_2, kernel, iterations=1)
        
        output_mask_1 = output_mask_1 * 0.005
        output_mask_2 = output_mask_2 * 0.005
        # blending
        alpha_1 = output_mask_1 / (output_mask_2 + output_mask_1)
        alpha_2 = output_mask_2 / (output_mask_2 + output_mask_1)

        # applying blend
        for i in range(3):
            resized_image_1[:, :, i] = resized_image_1[:, :, i] * alpha_1
            
        for i in range(3):
            resized_image_2[:, :, i] = resized_image_2[:, :, i] * alpha_2

        if DEBUG:
            cv2.imshow('Mask Image 1', output_mask_1)
            cv2.imshow('Mask Image 2', output_mask_2)
            cv2.imshow('Blended masks_1', alpha_1)
            cv2.imshow('Blended masks_2', alpha_2)
            cv2.imshow('Resized Image 1', resized_image_1)
            cv2.imshow('Resized Image 2', resized_image_2)
            # cv2.imshow("result_img1", result_img1)
            cv2.waitKey(0)

        #ROI = cv2.bitwise_and(mask_img2, mask_img1)            

        result = resized_image_1 + resized_image_2
        
        return result

def main():
    # Load the images to be stitched
    images = []
    images_dir = './panorama_images'
    images_path = os.path.join(images_dir, '*.jpeg')
    images = glob.glob(images_path)
    print(len(images))
    # Read the images
    img1 = cv2.imread(images[2])
    img2 = cv2.imread(images[1])
    img3 = cv2.imread(images[0])
    
    # img1 = cv2.resize(img1, (640, 480))
    # img2 = cv2.resize(img2, (640, 480))
    # img3 = cv2.resize(img3, (640, 480))
    
    # Convert images to grayscale
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    gray3 = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)

    # Detect ORB keypoints and descriptors
    orb = cv2.ORB_create()
    keypoints1, descriptors1 = orb.detectAndCompute(gray1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(gray2, None)
    keypoints3, descriptors3 = orb.detectAndCompute(gray3, None)

    #Draw keypoints on the images
    img1_keypoints = cv2.drawKeypoints(img1, keypoints1, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img2_keypoints = cv2.drawKeypoints(img2, keypoints2, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img3_keypoints = cv2.drawKeypoints(img3, keypoints3, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
   
    # Display the keypoints
    if DEBUG:
        cv2.imshow('Image 1 Keypoints', img1_keypoints)
        cv2.imshow('Image 2 Keypoints', img2_keypoints)
        cv2.imshow('Image 3 Keypoints', img3_keypoints)
        cv2.waitKey(0)

    # Create a BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors between the first two images
    matches1 = bf.match(descriptors1, descriptors2)

    # Sort matches based on distance
    matches1 = sorted(matches1, key=lambda x: x.distance)

    # Draw matches
    img_matches1 = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches1[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
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

    # Warp the images using the homography matrix
    panorama = warpImages(img2, img1, M1)

    # Display the panorama
    cv2.imshow('Panorama', panorama)
    cv2.waitKey(0)

    panorama_gray = cv2.cvtColor(panorama, cv2.COLOR_BGR2GRAY)
    # Detect ORB keypoints and descriptors in the panorama
    keypoints4, descriptors4 = orb.detectAndCompute(panorama_gray, None)

    # Match descriptors between the panorama and the third image AND sort matches based on distance
    matches2 = bf.match(descriptors4, descriptors3)
    matches2 = sorted(matches2, key=lambda x: x.distance)

    # Draw matches
    img_matches2 = cv2.drawMatches(panorama, keypoints4, img3, keypoints3, matches2[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
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
    
    panorama = warpImages(img3, panorama, M2)
    cv2.imshow('Final Panorama', panorama)
    cv2.waitKey(0)

    # Save the final panorama
    panorama_file = './panorama_new.jpg'
    cv2.imwrite(panorama_file, panorama)

    # Close all OpenCV windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()