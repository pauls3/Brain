# BFMC - Brain Project for Team Rebel Dynamics from University of Nevada, Las Vegas

This repo is Team Rebel Dynamics' attempt in the Bosch Future Mobility Challenge (BFMC) 2022. [This was forked from Bosch's Brain repo.](https://github.com/ECC-BFMC/Brain) The rest of the repos related to BFMC are located [here](https://github.com/ECC-BFMC/).

![alt text](https://github.com/pauls3/Brain/blob/master/images/me_resized.jpg)

## Lane Detection

For lane centering, we used a classical computer vision approach using the following steps:

  1. Get image from camera stream
  2. Convert to grayscale
  3. Get road region of interest (crop image)
  4. Blur image
  5. Thresholding to only extract bright white lines from the road
  6. Canny edge detection
  7. Hough lines (important since roads can either have solid or dashed lines)
  8. Average slope of hough lines to calculate steering error

![alt text](https://github.com/pauls3/Brain/blob/master/images/lane_centering.gif)



## Barricade Direction

![alt text](https://github.com/pauls3/Brain/blob/master/images/barricade.PNG)

The barricade is placed in a one-way two-lane road. It blocks one of the lanes and the car is supposed to go to the lane it is pointing at (left or right) and make the respected turn at the following intersection. Because we wanted to use the horizontal flip augmentation in the training the deep learning model, we made barricade one class. The goal of determining which direction the barricade was pointing is explained in the following steps:

  1. Get bounding box of barricade
  2. Expand the bounding box coordinates to take into account error
  3. Crop image based on bounding box (our region of interest)
  4. Convert to HSV
  5. Mask image to look for the red arrows
  6. Find contours to find the bounding boxes of the red arrows
  7. Perform voting based on cluster of center pixels on right side of the bounding box
  8. Determine majority vote as car gets closer to barricade and makes respected lane change if applicable

![alt text](https://github.com/pauls3/Brain/blob/master/images/barricade.gif)




## Object Detection

### Custom Dataset

Before training a deep-learning object-detection model, we first needed to create a custom dataset for this competition. Using the camera on the car, we took 4,560 images. The images were split between being "clean" (optimal lighting conditions, little to no added noise in the background), and being "noisy" or "realistic" (extreme lighting conditions, added clutter and human models to create a rich background. We ended up with 2,332 "clean" images and 2,228 "noisy" images. We then created 14,714 bounding box annotations. Extreme lighting conditions was artifically added since the dataset was created in a lab with very little natural light. We used a spotlight and RGB LED lights to create different lighting effects (e.g., different colors, glare) and we also reduced lighting for more variation. The goal was to make a robust model with the rich background and added noise from the light.

![alt text](https://github.com/pauls3/Brain/blob/master/images/dataset.PNG)
![alt text](https://github.com/pauls3/Brain/blob/master/images/class_distribution.png)

### Deep-Learning Model
We used the (SSD Mobilenet v2 COCO model in TensorFlow 1.14.)[https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md] We performed transfer learning on this model using our custom dataset using a 70 (train) / 20 (validation) / 10 (test) split. The splits were done by partitioning both the "clean" and "noisy" sets seperatly to create a balanced training, validation, and test splits. After training the model until the validation curved flattened, we tested the model and got the following results:
  * Average Precision @ IoU = 0.50:0.95 = 0.738
  * Average Precision @ IoU = 0.50      = 0.991
  * Average Precision @ IoU = 0.75      = 0.907

### OpenVINO
Running the model on the Raspberry Pi 4 only yielded around 2 frames-per-second. This was not good enough performance for us, so we used an Intel Neural Compute Stick 2 to run inference (like an GPU for the RaspPi). This boosted the fps to ~10-13. To run the TensorFlow model on the Intel Neural Compute Stick, we had to convert the model to (OpenVINO)[https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/overview.html].

![alt text](https://github.com/pauls3/Brain/blob/master/images/object_detection_0.gif)
![alt text](https://github.com/pauls3/Brain/blob/master/images/object_detection_1.gif)


## Competition Results

We mainly worked on getting object detection, using OpenVINO, and lane centering to work together in Python. It was a great experience to participate in this competition and I highly recommend anyone who is able to compete to take part in it!

Unfortunetly the car did not work as expected during the competition and I cannot compete again since I will be finishing my Masters' this year in 2022. However, I wish the best of luck for my teammates if they decide to compete next year!

![alt text](https://github.com/pauls3/Brain/blob/master/images/0911_BFMC-14.05.2022-NIC_1746-%C2%A9-Nicu-Cherciu.jpg)
