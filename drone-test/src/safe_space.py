import cv2
import numpy as np

def create_safe_zone(image, distance):
    
    # Return same image if no obstacles
    if not np.any(image): return image.astype(bool), image.astype(bool)

    # Load map as an image
    if image.dtype== bool: image = image.astype(np.uint8)
    
    #ret, original_img = cv2.threshold(image, 0, 1, cv2.THRESH_BINARY)

    img = image.copy()

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(distance,distance))  # Element for morph transformations

    full = cv2.dilate(img, kernel)

    #full_bool = np.logical_not(full.astype(bool))

    #ret, full = cv2.threshold(full, 0, 1, cv2.THRESH_BINARY_INV)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))  # Element for morph transformations

    gradient = cv2.dilate(full, kernel)

    full = full.astype(bool)

    gradient = gradient.astype(bool) * np.logical_not(full)

    

    return full, gradient


if __name__ == '__main__':
    
    img = np.zeros((800,800), dtype=np.uint8)
    # Set obstacle
    img[200:600, 600:700] = 255
    full, gradient = create_safe_zone(img, 50)
    gradient = (gradient * 255).astype(np.uint8)
    full = (full * 255).astype(np.uint8)
    #vor[200:600, 600:700] = 100
    cv2.imshow("Image", img)
    cv2.imshow("Gradient", gradient)
    cv2.imshow("Dilation", full)
    cv2.waitKey(0)
    #print(vor)