[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[image_1]: ./misc/threshold_image.jpeg
[image_2]: ./misc/warped_image.jpg
# Search and Sample Return Project
![alt text][image_0] 

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and it will give you first hand experience with the three essential elements of robotics, which are perception, decision making and actuation.  You will carry out this project in a simulator environment built with the Unity game engine.  

## The Simulator
The first step is to download the simulator build that's appropriate for your operating system.  Here are the links for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).  

You can test out the simulator by opening it up and choosing "Training Mode".  Use the mouse or keyboard to navigate around the environment and see how it looks.

## Dependencies
You'll need Python 3 and Jupyter Notebooks installed to do this project.  The best way to get setup with these if you are not already is to use Anaconda following along with the [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit). 


Here is a great link for learning more about [Anaconda and Jupyter Notebooks](https://classroom.udacity.com/courses/ud1111)

# Project writeup

## Notebook Analysis
### RGB Threshold 
The first step to achieve the autonomous navigation was to get the Rover to be able to distinguish between the obstacles, the samples and the navigable terrain. This was achieved by getting an image from the Rover's camera and processing said image. The image processing was very simple, an RGB threshold was applied to the image to get it to recognize the differents objects. Below is the function used for the RGB threshold

```python
def color_thresh(img, rgb_thresh_min=(170, 170, 170), rgb_thresh_max=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] >= rgb_thresh_min[0]) & (img[:,:,0] <= rgb_thresh_max[0]) & \
                   (img[:,:,1] >= rgb_thresh_min[1]) & (img[:,:,1] <= rgb_thresh_max[1]) & \
                   (img[:,:,2] >= rgb_thresh_min[2]) & (img[:,:,2] <= rgb_thresh_max[2]) 
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
 ```
 As you can see this function takes a min RGB and a max RGB threshold this is to help decrese overlapping with objects and so increasing fidelity. Below is the RGB threshold used for each of the objects
 
```python
navigable = color_thresh(warped, rgb_thresh_min=(170, 170, 170), rgb_thresh_max=(255, 255, 255))
    rock = color_thresh(warped, rgb_thresh_min=(130, 110, 0), rgb_thresh_max=(255, 230, 60))
    obstacle = color_thresh(warped, rgb_thresh_min=(0, 0, 0), rgb_thresh_max=(90, 90, 90))
```

Below is an image of what the output of said function would be, the white area is what the Rover sees, while the dark area is what it ignores

![alt text][image_1]

### Processing the image
Now that the Rover is able to distinguish between the different objects in the terrain, it is necessary for the Rover to tell exactly where they are located. For this, a perspective transform was applied to the image, this way all the objects could be located using their 'x' and 'y' positions. For the perspective transform a source and destination points were necessary so the image could be properly transformed, to achieve that the following was added.

```python
dst_size = 5
    bottom_offset = 6
    img_size = (img.shape[1], img.shape[0])
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[img_size[0]/2 - dst_size, img_size[1] - bottom_offset],
                      [img_size[0]/2 + dst_size, img_size[1] - bottom_offset],
                      [img_size[0]/2 + dst_size, img_size[1] - 2*dst_size - bottom_offset],
                      [img_size[0]/2 - dst_size, img_size[1] - 2*dst_size - bottom_offset],
                      ])
```
These points were passed on to the warped function along with the binary image which allowed for a successful transform. Functions used for this perspective transform are found in the Jupyter notebook. Note that the RGB threshold and the perspective transform can be applied on any order and the result would be the same. Below is what a return of the warped image would look like.

![alt text][image_2]

To simplify matters the RGB threshold and perspective transfomrs functions were called in a function called `process_image(img)` which takes as an input the image of the Rover's camera and applied all of the aforementioned functions.

### Constructing the worldmap
The `process_image(img)` function does one last thing, it compares the objects the Rover is seeing and an overlay of the world, this way a worldmap where we can see the navigable terrain, obstacles and rocks is constructed. For this we convert the thresholded images pixels values to rover-centric coords, this is done 3 times one for navigable terrain, one for the rocks, and one for the obstacles. This is done with the `rover_coords(binary_img)` function this functions returns the x and y position for each of the white pixels in the threshold image. Then using the `pix_to_world()` function the rover coordinates are converted to coordinates according to the world, this means that the x and y axis are changed to match where the rover is looking at, and the image itself is shrink according to the world size. Below is the code that was used for the transformation from rover cordinates to world coordinates.

```python
world_size = 200
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(x_pix, y_pix,
                                                        data.xpos[data.count], data.ypos[data.count], data.yaw[data.count],
                                                        world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(x_pix_rock, y_pix_rock,
                                                        data.xpos[data.count], data.ypos[data.count], data.yaw[data.count],
                                                        world_size, scale)
    obst_x_world, obst_y_world = pix_to_world(x_pix_obst, y_pix_obst,
                                                        data.xpos[data.count], data.ypos[data.count], data.yaw[data.count],
                                                        world_size, scale)
```

Note that the world size and scales are given and depend n both how big the worldmap and the rover are.

Lastly using the code below the worldmap that is stored in the rover is updated with the x and y coordinates for the obstacles, navigable terrain and rocks.

```python
    data.worldmap[obst_y_world, obst_x_world, 0] += 255
    data.worldmap[rock_y_world, rock_x_world, 1] += 255
    data.worldmap[navigable_y_world, navigable_x_world, 2] += 255  
```    

In the code above the worldmap is stored in `data.worldmap` this is only used in the Jupyter notebook as in the actual code the appropiate way to update the wordlmap is calling `Rover.worldmap`

After `processing_image()` is called in the jupyter notebook it creates a short movie using the data, you can find this movie in the output folder of the repository.


## Autonomous Navigation and Mapping
The functions that are called when drive_rover.py is executed are `perception_step()` and `decision_step()`

### Perception Step
The `perception_step()` function is very similar to the `process_image()` found in the jupyter notebook. The main differences is that the code below was added

```python
    Rover.vision_image[:,:,0] = obstacle * 255
    Rover.vision_image[:,:,1] = rock * 255
    Rover.vision_image[:,:,2] = navigable * 255
```

This code updates the image in the simulator that shows us what the Rover is seeing, the obstacles, rock and navigable here are the perspective transformed after the RGB is applied for each of them, they are multiplied by 255 to make them more visible against a black background.

Another change was the code shown below:

```python
    distance, angle = to_polar_coords(x_pix, y_pix)
    Rover.nav_dists = distance
    Rover.nav_angles = angle

    # Update Rocks pixel distances and angles
    distance_rock, angle_rock = to_polar_coords(x_pix_rock, y_pix_rock)
    Rover.rock_dists = distance_rock
    Rover.rock_angles = angle_rock
```

This code send the rover not only the angles and distance to the navigable terrain but also the angles and distances to the rock samples, this will alow the ROver to more easily get to the rocks for pickup.

### Decision Step
This function is called after the perception function, so the Rover already knows what is in front of him. Some changes were made to the given code, they were made so the Rover could go near a rock and pick it up

```python
# Gets the Rover unstuck if the picking up flag is 1
    if Rover.picking_up:
        return Rover
# Check if a rock is in sight
    if len(Rover.rock_angles) > 1:
        # Checks if the Rover is already in picking up range
        if Rover.near_sample:
            Rover.brake = 20
            Rover.pick_up = True
        # If not then reduce speed so it can steadily approach
        elif Rover.vel > 0.5:
            Rover.throttle = 0
        elif Rover.vel < 0.5:
            Rover.throttle = Rover.throttle_set
        # Keep steering towards the rock
        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
```

This piece of code was placed before the Rover checks for navigable terrain, this is so the Rover gives priority to the rock samples.

### Todo:
The Rover can be further improved in several ways, one of them would be for the Rover to return to its initial position after all the samples have been found. This can be donde as the Rover saves its x and y position so at the start that position can be saved to go back to it earlier.

The impact detection can also be improved as the Rover sometimes crashes against an obstacle and has a hard time getting out.

Smarter steering was also considered as sometimes the Rover keeps going around in a circle for a while.
