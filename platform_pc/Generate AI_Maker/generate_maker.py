import cv2 as cv
import numpy as np

# Load the predefined dictionary
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

# Generate the marker
marker_size = 200  # Size in pixels
marker_id = 3     # ID of the marker to generate
markerImage = cv.aruco.generateImageMarker(dictionary, marker_id, marker_size)
cv.imwrite("marker3.png", markerImage)