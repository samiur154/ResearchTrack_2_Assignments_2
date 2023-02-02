# RT2_Assignment2
## Jupyter notebook package:
This package includes Jupyter Notebooks, which act as a user interface node and assist in controlling the robot in the gazebo environment. This node will have a graphical interface with a Jupyter notebook for starting and stopping the robot, controlling the robot in the desired direction, and also analyzing various plots. In the action package, the user interface is simply a command line interface through which the user can "start" or "stop" the robot.

## Jupyter Notebook desription:
Plots, buttons is for controlling the robot, and displays for various analysis data on the robot's state are all included in the Jupyter Notebook. The buttons and plots that this node can offer are listed below.

### Start,Stop,foreward,backward,left,right buttons:
The robot may be manually started and stopped with this button, and it can also be moved in any direction using the forward, backward, left, and right buttons. These buttons provide as a means of requesting customer service.

![but_jupy](https://user-images.githubusercontent.com/80621864/154955314-6963db8c-23d2-49e4-811a-2452e962a76c.jpg)

### Robot's moving position in real time:
it is a live plot that displays the robot's condition in real time. It is an x, y plot that is used to indicate the robot's position as well as the direction it is travelling in the gazebo environment. Robot motion will be shown by a black line that looks like a diamond.
![map_jupy](https://user-images.githubusercontent.com/80621864/154955780-9474e2e9-7205-48a8-a9d6-31f4c7c4ab6a.jpg)

### Velocity visualization plot:
This figure makes it easier to see how the **cmd_vel** compares to the **odom(actual velocity)**. Both in linear and angular positions, it is visible.  against the 

![Screenshot 2022-02-21 114438](https://user-images.githubusercontent.com/80621864/154956187-c5c24725-6045-499d-8544-8cb020882c2d.jpg) ![Screenshot 2022-02-21 114414](https://user-images.githubusercontent.com/80621864/154957806-3044a32b-0ba2-490a-8863-b9f7fdbf1c4c.jpg) ![Screenshot 2022-02-21 114343](https://user-images.githubusercontent.com/80621864/154956251-a9522580-93a5-4e4a-8cb1-444f4b790fdc.jpg) ![vel_jupy](https://user-images.githubusercontent.com/80621864/154956271-f240dbb2-2922-44da-ba1d-ef6c77697306.jpg)

### Bar plot:
Bar plot is used for showing the number of target reached and the number of target that as cancelled by the user.

![bar_tar](https://user-images.githubusercontent.com/80621864/154956539-3706afeb-484f-4db5-8399-a2c252391a55.jpg)

### Histogram plot:
The histogram plot shows the time taken to reach the target by the robot wn it is reached the goal.

![time_jupy](https://user-images.githubusercontent.com/80621864/154956810-0b0ae5db-65ec-4cbd-af07-1082146131b8.jpg)

## Running the package:
- In the first tab:
run the command below to launch the simulation and all the required nodes.
```roslaunch rt2_assignment2 sim.launch```
- In the second tab:
run the command to open jupyter notebook.
```jupyter notebook --allow-root```
and open the notebook named as **jupyter_505150.ipynb** .




https://samiur154.github.io/ResearchTrack_2_Assignments_2/



