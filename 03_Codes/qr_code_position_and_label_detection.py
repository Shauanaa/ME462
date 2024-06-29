import cv2
import numpy as np
from qreader import QReader
from typing import Dict, Tuple

def detect_qr_codes(image: np.ndarray) -> Tuple[Dict[str, Tuple[float, float]], np.ndarray]:
    """
    Detect and decode QR codes in the given image, and return their data and coordinates.

    This function uses the QReader library to detect and decode QR codes in the input image.
    It then plots the centroids of the detected QR codes on the image and adds the decoded
    data as text next to each centroid.

    Args:
        image (np.ndarray): Input image in BGR format.

    Returns:
        Tuple[Dict[str, Tuple[float, float]], np.ndarray]: A tuple containing:
            - A dictionary with QR code data as keys and their coordinates (x, y) as values.
            - The input image with QR codes and centroids plotted.

    Example:
        >>> import cv2
        >>> image = cv2.imread("qr_code_image.jpg")
        >>> qr_data, annotated_image = detect_qr_codes(image)
        >>> print(qr_data)
        >>> cv2.imshow("Detected QR Codes", annotated_image)
        >>> cv2.waitKey(0)
        >>> cv2.destroyAllWindows()
    """
    # Convert BGR to RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Initialize QReader
    qreader = QReader()

    # Detect and decode QR codes
    decoded_codes = qreader.detect_and_decode(image=image_rgb)
    decode_data = qreader.detect(image=image_rgb)

    # Create a dictionary to store QR code data and coordinates
    qr_data = {}

    for index, code in enumerate(decoded_codes):
        qr_data[code] = decode_data[index]['cxcy']

    # Plot centroids on the image
    for code, centroid in qr_data.items():
        x, y = map(int, centroid)
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)  # Draw a green circle at the centroid
        cv2.putText(image, code, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # Add QR code data as text

    return qr_data, image

# Example usage:
# image = cv2.imread("your_image.jpg")
# qr_data, annotated_image = detect_qr_codes(image)
# print("QR Code Data and Coordinates:")
# print(qr_data)
# cv2.imshow("Detected QR Codes", annotated_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
