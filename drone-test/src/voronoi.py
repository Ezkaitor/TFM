#!/usr/bin/env python

import numpy as np
import cv2

def generate_voronoi(original_img):
    ''' Loads a binary map and returns the voronoi representation'''
    
    # Load map as an image
    ret, original_img = cv2.threshold(original_img, 0, 1, cv2.THRESH_BINARY_INV)

    # Resize the image for showing purposes
    #mult = 10
    #dim = (original_img.shape[1] * mult, original_img.shape[0] * mult)
    #original_img = cv2.resize(original_img, dim, interpolation = cv2.INTER_AREA)

    img = original_img.copy()

    size = np.size(img)
    skel = np.zeros(img.shape,img.dtype)

    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))  # Element for morph transformations
    #img = cv2.erode(img, element)

    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))  # Element for morph transformations
    done = False


    # Skelitization
    while not done:
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        
        # Stop when the image is fully eroded
        zeros = size - cv2.countNonZero(img)
        if zeros==size:
            done = True

    
    # Image showing

    #cv2.imshow("skel",skel)
    #cv2.imshow("image", original_img)o999999999999999999999999999999999pooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
    # Free spaces = 0
    ret, final_img = cv2.threshold(skel, 0, 1, cv2.THRESH_BINARY_INV)
    return final_img

if __name__ == '__main__':
    img = np.zeros((800,800), dtype=np.uint8)
    # Set borders to one
    img[0,:] = 255
    img[-1,:] = 255
    img[:, 0] = 255
    img[:, -1] = 255
    # Set obstacle
    img[200:600, 600:700] = 255
    vor = generate_voronoi(img)
    vor = (vor * 255).astype(np.uint8)
    cv2.imshow("Image", img)
    cv2.imshow("Voronoi", vor)
    cv2.waitKey(0)
    #print(vor)