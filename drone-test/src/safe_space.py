import cv2
import numpy as np

def create_safe_zone(image, distance):

    # Load map as an image
    ret, original_img = cv2.threshold(image, 0, 1, cv2.THRESH_BINARY_INV)

    img = original_img.copy()

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(distance,distance))  # Element for morph transformations

    gradient = cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel) # Get the safe zone

    full = cv2.erode(img, kernel)

    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(2,2))  # Element for morph transformations

    
    gradient = cv2.morphologyEx(gradient, cv2.MORPH_GRADIENT, kernel) # Delete internal map
    
    gradient *= img

    return np.logical_not(full.astype(bool)), gradient.astype(bool)


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