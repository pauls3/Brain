# BFMC - Brain Project for Team Rebel Dynamics from University of Nevada, Las Vegas

This repo is Team Rebel Dynamics' attempt in the Bosch Future Mobility Challenge 2022.

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


## Competition Results

We mainly worked on getting object detection, using OpenVino, and lane centering to work together in Python. It was a great experience to participate in this competition and I highly recommend anyone who is able to compete to take part in it!

Unfortunetly the car did not work as expected during the competition and I cannot compete again since I will be finishing my Masters' this year in 2022. However, I wish the best of luck for my teammates if they decide to compete next year!

![alt text](https://github.com/pauls3/Brain/blob/master/images/0911_BFMC-14.05.2022-NIC_1746-%C2%A9-Nicu-Cherciu.jpg)
