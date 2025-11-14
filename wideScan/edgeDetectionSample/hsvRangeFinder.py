import cv2
import numpy as np
from typing import Any

# This function does nothing but is required by the trackbar
def nothing(x: Any) -> None:
    pass

# PUT IMAGE FILE PATHS HERE TO FIND THE SUITABLE HSV RANGE FOR THEM:
IMAGE_PATH = '../Iarc_photo_folder/mine_in_grass templates_(combo-2).png'
#For more photos: [First Photo Here, '../Iarc_photo_folder/mine_in_grass templates_(combo).png']


img = cv2.imread(IMAGE_PATH)
if img is None:
    print(f"Error: Could not load image from {IMAGE_PATH}")
    exit()

# Resize for display if it's too big
h, w = img.shape[:2]
if w > 1200:
    scale = 1200 / w
    img = cv2.resize(img, (int(w * scale), int(h * scale)))

# Convert to HSV
hsv: np.ndarray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# --- 2. Create Trackbar Window ---
cv2.namedWindow('Trackbars')
cv2.createTrackbar('H_min', 'Trackbars', 90, 179, nothing)
cv2.createTrackbar('H_max', 'Trackbars', 130, 179, nothing)
cv2.createTrackbar('S_min', 'Trackbars', 40, 255, nothing)
cv2.createTrackbar('S_max', 'Trackbars', 150, 255, nothing)
cv2.createTrackbar('V_min', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('V_max', 'Trackbars', 220, 255, nothing)

print("\n--- HSV Calibration Tool ---")
print("Adjust the sliders until only the mines are white in the 'Mask' window.")
print("Press 's' to save and calculate.")
print("Press 'q' to quit without saving.")
print("---------------------------------")

final_mask = None

while True:
    # --- 3. Read Trackbar Values ---
    h_min = cv2.getTrackbarPos('H_min', 'Trackbars')
    h_max = cv2.getTrackbarPos('H_max', 'Trackbars')
    s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
    s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
    v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
    v_max = cv2.getTrackbarPos('V_max', 'Trackbars')

    # Create the bounds
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])

    # --- 4. Create and Show Mask ---
    mask: np.ndarray = cv2.inRange(hsv, lower_bound, upper_bound)
    # Apply the mask to the original image
    result = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow('Original Image', img)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('s'):
        final_mask = mask
        print("Mask saved! Calculating stats...")
        break

cv2.destroyAllWindows()

# --- 5. Calculate and Print Stats ---
if final_mask is not None:
    # Check if we actually selected any pixels
    if np.count_nonzero(final_mask) > 0:
        
        # This is the magic function!
        # It calculates mean and std dev, but only for pixels
        # specified in the mask.
        mean, std_dev = cv2.meanStdDev(hsv, mask=final_mask)
        
        print("\n--- Statistics for Selected Pixels ---")
        print(f"      H      S      V")
        print(f"Mean: {mean.flatten().round(2)}")
        print(f"StdDev: {std_dev.flatten().round(2)}")

        # --- 6. Generate New Bounds ---
        # A common rule is to use 2-3 standard deviations
        # We must clip values to be in the valid 0-255 range
        
        stats = np.array([mean, std_dev]).flatten()
        
        # Calculate bounds using 2 standard deviations
        # Note: 179 is max for H, 255 for S and V
        lower = np.array([stats[0] - 2 * stats[3],  # H_mean - 2*H_std
                          stats[1] - 2 * stats[4],  # S_mean - 2*S_std
                          stats[2] - 2 * stats[5]]) # V_mean - 2*V_std
                          
        upper = np.array([stats[0] + 2 * stats[3],  # H_mean + 2*H_std
                          stats[1] + 2 * stats[4],  # S_mean + 2*S_std
                          stats[2] + 2 * stats[5]]) # V_mean + 2*V_std

        # Clip the values to their valid ranges
        lower_bound_final = np.clip(lower, [0, 0, 0], [179, 255, 255]).astype(int)
        upper_bound_final = np.clip(upper, [0, 0, 0], [179, 255, 255]).astype(int)

        print("\n--- Suggested New Bounds for Your Program ---")
        print(f"lower_bound = np.array({lower_bound_final.tolist()})")
        print(f"upper_bound = np.array({upper_bound_final.tolist()})")
        
    else:
        print("No pixels were selected in the mask. No stats to calculate.")
else:
    print("Operation cancelled. No stats calculated.")