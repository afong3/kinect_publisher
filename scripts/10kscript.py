# import client from inference_sdk
from inference_sdk import InferenceHTTPClient
# import os to get the API_KEY from the environment
import os
import cv2

# set the project_id, model_version, image_url
project_id = "10k"
model_version = 4
image_url = "8.jpg"

# create a client object
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="T4iCckQT5pFVF7LmXboQ"
)

# run inference on the image
results = CLIENT.infer(image_url, model_id=f"{project_id}/{model_version}")

# print the results
print(results)

# Load your image (replace 'image_path.jpg' with the path to your image)
image = cv2.imread(image_url)

# Iterate through the detections
for detection in results["predictions"]:
    x, y, width, height = detection['x'], detection['y'], detection['width'], detection['height']
    confidence, class_name = detection['confidence'], detection['class']

    # Draw bounding box
    cv2.rectangle(image, (int(x), int(y)), (int(x + width), int(y + height)), (0, 255, 0), 2)

    # Add label and confidence score
    label = f'{class_name}: {confidence:.2f}'
    cv2.putText(image, label, (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the image
cv2.imshow('Object Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()