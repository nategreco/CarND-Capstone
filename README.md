# **Self-Driving Car Nanodegree Capstone Project**

### Using ROS to program a real self-driving car

---

**Team Members**

| Name                 | Email                            | Slack Handle     | Node             |
|:--------------------:|:--------------------------------:|:----------------:|:----------------:|
| Nathan Greco         | nathan.greco@gmail.com           | @nategreco       | waypoint_updater |
| Charles Faivre       | chuck@ratsnestinc.com            | @cpfaivre        | dbw_node         |
| John Novotny         | john.novotny@gmail.com           | @jnovotny        | twist_controler  |
| Mohamed Hussien      | eng.mohamedhussien1991@gmail.com | @mohamed_hussien | tl_detector      |
| Virendra Kumar Anand | vkanand1976@gmail.com            | @vkanand         | tl_classifier    |

**Capstone Project Goals**

The goals / steps of this project are the following:

* Smoothly follow waypoints in the simulator.
* Respect the target top speed set for the waypoints' 'twist.twist.linear.x' in 'waypoint_loader.py'. Be sure to check that this is working by testing with different values for kph velocity parameter in '/ros/src/waypoint_loader/launch/waypoint_loader.launch'. If your vehicle adheres to the kph target top speed set here, then you have satisfied this requirement.
* Stop at traffic lights when needed.
* Stop and restart PID controllers depending on the state of '/vehicle/dbw_enabled'.
* Publish throttle, steering, and brake commands at 50hz.
* Launch correctly using the launch files provided in the capstone repo. **Please note that we will not be able to accomodate special launch instructions or run additional scripts from your submission to download files.** The 'launch/styx.launch' and 'launch/site.launch' files will be used to test code in the simulator and on the vehicle respectively. The submission size limit for this project has been increased to 2GB.

[//]: # (Image References)

---

### Discussion

#### 1. Implementation

##### a. Waypoint Updater (partial)

ToDo

##### b. DBW Node

ToDo

##### c. Twist Controller

ToDo

##### d. Traffic Light Detector

Traffic Light Detector is the node that responsible for:
* Detect the traffic light from the image captured by camera
* Send the detected traffic light to tl_classifier function to classify it
* Publish the traffic light state and location

This node subscribes to four topics:
* `/base_waypoints` provides the complete list of waypoints for the course.
* `/current_pose` can be used to determine the vehicle's location.
* `/image_color` which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
* `/vehicle/traffic_lights` provides the (x, y, z) coordinates of all traffic lights.

This node publishes the index of the waypoint for nearest upcoming red light's stop line to a single topic:
* `/traffic_waypoint`

What I did:
* I implemented `get_closest_waypoint` function to use it to get the nearest front waypoints to the car, the nearest traffic light and the nearest stop line
* I implemented `process_traffic_lights` function to get the nearest traffic light and its stop line and returns the traffic light state and its stop line location
* I implemented `get_light_state` function to detect the traffic light from the image and sends the detected one to the classifier

Info about the detection model:
* I used the tiny_YOLO_v2 architecture as it's suitable for real time applications
* The model consists of 9 convolutional layers
* I used the pretrained model on COCO dataset as it already has a traffic_light class
* I used the weights provided by the dark_net from this but I got nans as result of any input, And when I got inside the weights file I found a zero was added by fault to the beginning to the file, So I removed it and became the first one to be able to use this pretrained model
* I used non-maximum-suppression on the resulted tensor to get the detection boxes of the wanted class
* The file `object_detection.ipynb` of Building the architecture and loading the weights is in `yolo detector model` folder

Here are some results of the detection model:<br>
<img src="./ros/src/tl_detector/yolo detector model/test_images_output/1.jpg" alt="original" height=150px>
<img src="./ros/src/tl_detector/yolo detector model/test_images_output/left0011.jpg" alt="original" height=150px>
<img src="./ros/src/tl_detector/yolo detector model/test_images_output/left0003.jpg" alt="original" height=150px >

<img src="./ros/src/tl_detector/yolo detector model/test_images_output/left0183.jpg" alt="original" height=150px>
<img src="./ros/src/tl_detector/yolo detector model/test_images_output/left0701.jpg" alt="original" height=150px>
<img src="./ros/src/tl_detector/yolo detector model/test_images_output/left0282.jpg" alt="original" height=150px>

* For more examples check the `yolo detector model` folder


##### e. Traffic Light Classifier

Traffic Light Classifier

Traffic light classifier is responsible for classifying the passed images as Red, Green or Yellow state of traffic light. Traffic light detector module detects an object as traffic light and saves the image of detected traffic light. It calls Traffic light classifier and passes the image of detected traffic light and gets back a label corresponding to the color state of the traffic light. Traffic light is a re-trained Inception classifier on the traffic light images extracted from Bosch Small Traffic Lights Dataset. Different size and brightness images were used for the training. Also some of the images were little angled. The inception classifier as explained in below github has been used for 5000 steps. 

 https://github.com/llSourcell/tensorflow_image_classifier - automatic!

After the end of 5000 steps, the training process resulted in training accuracy of 91.2% and validation accuracy of 93%. After that few random images of traffic signs were passed that gave below results.

|Image   | Actual Color   |  Predicted  | Probability of Prediction   |  Comment    |
|:------:|:--------------:|:-----------:|:---------------------------:|:----------:|  
|<img src="./ros/src/tl_detector/light_classification/tl_sample/0r1.jpeg" alt="original" height=150px>| Yellow | Yellow |0.76714|	|
|<img src="./ros/src/tl_detector/light_classification/tl_sample/0r3.jpeg" alt="original" height=150px>| Red    | Red    |0.67141| |
|<img src="./ros/src/tl_detector/light_classification/tl_sample/0s1.jpeg" alt="original" height=150px>| Red    | Red    |0.58037| |
|<img src="./ros/src/tl_detector/light_classification/tl_sample/0s2.jpeg" alt="original" height=150px>| Yellow | Red    |0.53224| |
|<img src="./ros/src/tl_detector/light_classification/tl_sample/0s3.jpeg" alt="original" height=150px>| Green  | Green  |0.7778 |Picked up safer option.|
|<img src="./ros/src/tl_detector/light_classification/tl_sample/just_traffic_light_0002.jpg" alt="original" height=150px>| Green | Red    |0.38152|No clear winner, slightly inclined towards red.|
|<img src="./ros/src/tl_detector/light_classification/tl_sample/just_traffic_light_0051.jpg" alt="original" height=150px>| Green| Green  |0.67349| |
|<img src="./ros/src/tl_detector/light_classification/tl_sample/just_traffic_light_0127.jpg" alt="original" height=150px>| Yellow| Yellow |0.5037| |
|<img src="./ros/src/tl_detector/light_classification/tl_sample/just_traffic_light_0410.jpg" alt="original" height=150px>| Yellow| Red |0.48372|No clear winner, slightly inclined towards red. |
|<img src="./ros/src/tl_detector/light_classification/tl_sample/just_traffic_light_0469.jpg" alt="original" height=150px>| Red| Yellow |0.41307|No clear winner. Picked up Yellow instead of Red.|
|<img src="./ros/src/tl_detector/light_classification/tl_sample/just_traffic_light_0601.jpg" alt="original" height=150px>| Green| Green |0.960062||
|<img src="./ros/src/tl_detector/light_classification/tl_sample/left0000.jpg" alt="original" height=150px>| Green| Green |0.84519||
|<img src="./ros/src/tl_detector/light_classification/tl_sample/left0003.jpg" alt="original" height=150px>| Green| Green |0.74188||
|<img src="./ros/src/tl_detector/light_classification/tl_sample/left0011.jpg" alt="original" height=150px>| Yellow| Red |0.42038|No clear winner, slightly inclined towards red.|


The above shows that in the simulator images, Yellow was mostly picked as Red which is safer option. To speed up the inference process, model was loaded only once during when TLClassifier object was created. In case the inference was giving probability of less than 50% on any image then that detection was not considered as good and defaulted to red light detection for a safer option.



##### f. Waypoint Updater (full)

ToDo


#### 2. Results

ToDo
