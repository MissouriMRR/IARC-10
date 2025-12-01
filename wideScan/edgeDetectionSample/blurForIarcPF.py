import cv2
import numpy as np
import sys
from typing import List, Tuple, Optional, Any # Import necessary types

# Define a type alias for OpenCV/Numpy images for clarity
# An image is just a multi-dimensional numpy array
ImageType = np.ndarray

#Creating the list of photos to run through

files: List[str] = ['../Iarc_photo_folder/mine_in_grass templates_(combo-2).png',
                   '../Iarc_photo_folder/mine_in_grass templates_(combo).png']

"""['../Iarc_photo_folder/mine_in_grass_templates.png',
         '../Iarc_photo_folder/mine_in_grass_templates1.png',
         '../Iarc_photo_folder/mine_in_grass_templates2.png',
         '../Iarc_photo_folder/mine_in_grass_templates3.png',
         '../Iarc_photo_folder/mine_in_grass_templates4.png',
         '../Iarc_photo_folder/mine_in_grass_templates_prime.png',
         '../Iarc_photo_folder/mine_in_grass_templates_prime2.png',
         '../Iarc_photo_folder/mine_in_grass_templates_prime3.png']"""

# array to store coordinate(tuple) values to be used for coordinate finding
# A list of tuples, where each tuple contains two integers (x, y)
coordinate_list: List[Tuple[int, int]] = []

# HSV color range for the grey-blue mines.
lower_bound: ImageType = np.array([90, 40, 100])
upper_bound: ImageType = np.array([130, 150, 220])

# Kernel for cleaning the mask
kernel: ImageType = np.ones((5, 5), np.uint8)


#Loop Through Each File

for file_path in files:
    # Load the image
    # cv2.imread returns an ImageType (np.ndarray) on success or None on failure
    img: Optional[ImageType] = cv2.imread(file_path)

    # CRITICAL: Check if the image was loaded successfully
    if img is None:
        print(f"Error: Could not load image from {file_path}")
        print("Skipping to the next file.\n")
        continue  # Go to the next iteration of the for loop

    # Create a copy to draw results on
    output_image: ImageType = img.copy()

    # Switches from RGB to Hue, Saturation, and Value
    hsv: ImageType = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Create the binary mask (only allows for fully white/black pixels)
    mask: ImageType = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Clean the mask
    mask_closed: ImageType = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_cleaned: ImageType = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

    # Find and Filter Mines
    # findContours returns a tuple: (list_of_contours, hierarchy)
    contours: Tuple[ImageType, ...]
    hierarchy: ImageType
    contours, hierarchy = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mine_count: int = 0
    
    # Each 'cnt' is a single contour, which is an np.ndarray of points
    cnt: ImageType 
    for cnt in contours:
        # Filter by area to remove small noise (like watermarks)
        area: float = cv2.contourArea(cnt)
        print(f"Found contour with area: {area}")

        # This filter is more reliable than W/H and (W*H)
        if area > 80 and area < 5000: 
            mine_count += 1
            
            # Get bounding box and draw it
            # boundingRect returns (x, y, w, h) as integers
            x: int
            y: int
            w: int
            h: int
            x, y, w, h = cv2.boundingRect(cnt)
            
            cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 0, 255), 3) # Changed thickness to 3
            center_x: int = x + int(w / 2)
            center_y: int = y + int(h / 2)
            coordinate_list.append((center_x, center_y))
            
            # Put a label
            cv2.putText(output_image, f"Mine #{mine_count}", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    print(f"Found {mine_count} potential mines in: {file_path}")

    # --- 5. Resize and Display Results ---
    
    # Calculate new size (30% of original)
    try:
        newSize: Tuple[int, int] = (int((img.shape[1])*0.3), int((img.shape[0])*0.3))
        resized_detections: ImageType = cv2.resize(output_image, newSize, interpolation=cv2.INTER_LINEAR)
        resized_mask: ImageType = cv2.resize(mask_cleaned, newSize, interpolation=cv2.INTER_LINEAR)
    except Exception as e:
        print(f"Error resizing image {file_path}: {e}")
        continue

    # Show the images
    cv2.imshow('Detections', resized_detections)
    cv2.imshow('Mask (for debugging)', resized_mask)

    #printing coordinates
    coordinates: np.ndarray = np.array(coordinate_list) # This is an N-by-2 array
    print("\n--- Summary of Mine Coordinates ---")
    if coordinates.shape[0] > 0:
        print(f"Found a total of {coordinates.shape[0]} mines across all files.")
        print("Coordinates (center x, center y):")
        print(coordinates)

    # Wait for 'q' to Move to Next Image 
    print("Press 'q' in any window to proceed to the next image...")
    while(True):
        # Check for a key press
        key: int = cv2.waitKey(1) & 0xFF
        # If 'q' is pressed, break the inner loop
        if key == ord('q'):  
            break 

    else:
        print("No mines were found in any files.")
            
    # Close the windows for the current image before opening the next
    cv2.destroyAllWindows()

print("\nFinished processing all files.")