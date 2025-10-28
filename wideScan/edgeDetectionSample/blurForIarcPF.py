import cv2
import numpy as np
import sys

# --- 1. Define File List and Detection Parameters ---

files = ['../Iarc_photo_folder/mine_in_grass_templates.png',
         '../Iarc_photo_folder/mine_in_grass_templates1.png',
         '../Iarc_photo_folder/mine_in_grass_templates2.png',
         '../Iarc_photo_folder/mine_in_grass_templates3.png',
         '../Iarc_photo_folder/mine_in_grass_templates4.png',
         '../Iarc_photo_folder/mine_in_grass_templates_prime.png',
         '../Iarc_photo_folder/mine_in_grass_templates_prime2.png',
         '../Iarc_photo_folder/mine_in_grass_templates_prime3.png']

# HSV color range for the grey-blue mines.
# You may need to tune these if lighting is different in other photos.
lower_bound = np.array([90, 40, 100])
upper_bound = np.array([130, 150, 220])

# Kernel for morphological operations (cleaning the mask)
kernel = np.ones((5, 5), np.uint8)


# --- 2. Loop Through Each File ---

for file_path in files:
    # Load the image
    img = cv2.imread(file_path)

    # CRITICAL: Check if the image was loaded successfully
    if img is None:
        print(f"Error: Could not load image from {file_path}")
        print("Skipping to the next file.\n")
        continue  # Go to the next iteration of the for loop

    # Create a copy to draw results on
    output_image = img.copy()

    # --- 3. HSV Color Segmentation (The Core Logic) ---
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Create the binary mask
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Clean the mask
    mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_cleaned = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

    # --- 4. Find and Filter Mines ---
    contours, _ = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mine_count = 0
    for cnt in contours:
        # Filter by area to remove small noise (like watermarks)
        area = cv2.contourArea(cnt)
        
        # This filter is more reliable than W/H and (W*H)
        if area > 500 and area < 10000: 
            mine_count += 1
            
            # Get bounding box and draw it
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 0, 255), 3) # Changed thickness to 3
            
            # Put a label
            cv2.putText(output_image, f"Mine #{mine_count}", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    print(f"Found {mine_count} potential mines in: {file_path}")

    # --- 5. Resize and Display Results ---
    
    # Calculate new size (30% of original)
    try:
        newSize = (int((img.shape[1])*0.3), int((img.shape[0])*0.3))
        resized_detections = cv2.resize(output_image, newSize, interpolation=cv2.INTER_LINEAR)
        resized_mask = cv2.resize(mask_cleaned, newSize, interpolation=cv2.INTER_LINEAR)
    except Exception as e:
        print(f"Error resizing image {file_path}: {e}")
        continue

    # Show the images
    cv2.imshow('Detections', resized_detections)
    cv2.imshow('Mask (for debugging)', resized_mask)

    # --- 6. Wait for 'q' to Move to Next Image ---
    print("Press 'q' in any window to proceed to the next image...")
    while(True):
        # Check for a key press
        key = cv2.waitKey(1) & 0xFF
        # If 'q' is pressed, break the inner loop
        if key == ord('q'):  
            break 
            
    # Close the windows for the current image before opening the next
    cv2.destroyAllWindows()

print("\nFinished processing all files.")