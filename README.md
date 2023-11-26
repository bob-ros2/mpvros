# ROS Node wrapped around popular MPV video player
With this ros node an instance of MPV video player can be started using the 
configured ROS parameter.\
Additionaly a MPV ipc-server.sock will be opened which can be accessed 
directly or via a topic. Responses from MPV player can be received and are 
also published via a topic.\
Use the full MPV functionality within ROS to add additional dynamic visuals
 for the robot operator.

A lot of functions are availabe with MPV player. Here are some of them:
* Play video, audio and streams
* Sending commands to the player during playing
* Retrieve state information of the player
* Scaling of video
* Positioning of the video window
* Remove of the window borders or even window caption
* Playing just part of the Video
* OSD functions
* Configurable keyboard controls
* ...

Additional information can be found here:
* https://mpv.io
* https://mpv.io/manual/stable

## Dependencies
```bash
# install MPV video player
sudo apt-get install mpv
```

## Setup package ##
```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/mpvros.git
cd ..
colcon build
. install/setup.bash
```

# ROS Node MPV

## Usage
```bash
# show scaled video without border centered on the screen
ros2 run mpvros mpv --ros-args -p "args:=['/home/ros/video.mkv','--no-border',
'--window-scale=0.5','--geometry=50%:50%']"

# run with custom ipc-server socket path
ros2 run mpvros mpv --ros-args -p "ipc_server:=/home/ros/.mpvros.sock" -p 
"args:=['/home/ros/video.mkv']"
```

## Node Parameter
> ~ipc_server (string, default: "/tmp/mpvros.sock")\
Server socket to be used for ipc communication.

> ~args (string_array, default: [])\
Command line options to be passed to MPV player. Currently spaces are not 
allowed in the array items. See also https://mpv.io/manual/stable

## Subscribed Topics
> ~ipc_in (std_msgs/String)\
Incoming ipc data to be forwarded to MPV ipc-server socket.

## Published Topics
> ~ipc_out (std_msgs/String)\
Outputs received ipc data from MPV ipc-server socket.

# ROS Node Title

The title node updates periodically the current running MPV player title. This 
works for single file and for playlists. The current title is published 
to a topic.

## Usage
```bash
# start title node
ros2 run mpvros title

# start with remapping
ros2 run mpvros title --ros-args -r ipc_in:=/mpv1/ipc_in -r ipc_out:=/mpv1/ipc_out
```

## Subscribed Topics

> ~ipc_in (std_msgs/String)\
Incoming ipc data from MPV player.

## Published Topics

> ~title (std_msgs/String)\
Outputs current title.

> ~ipc_out (std_msgs/String)\
Outgoing ipc data to request current title.

# ROS Node Scheduler

The Scheduler node can be used as central point to start and 
maintain running MPV playing orders.

## Usage
```bash
# start scheduler to wait for incoming MPV playing orders
ros2 run mpvros scheduler

# publish a playing order to the scheduler topic
ros2 topic pub --once /schedule std_msgs/msg/String "{data: '/your/path/vid.mp4 --no-border --window-scale=0.4 --geometry=50%:50% --loop=inf'}"

# send playing order via action service, if parameter id is filled a single instance with this id will be forced
ros2 action send_goal /SuperBob/schedule mpvros/action/Schedule "{id: bob, args: /path/to/video.mkv --loop=inf --geometry=50%:50% --window-scale=0.4 }"

```

## Subscribed Topics
> ~schedule (std_msgs/String)\
Receive schedule order.

## Published Topics
> ~done (std_msgs/String)\
Publishes MD5 sum of the schedule order which finished the job.

# Launch Files

## MPV.LAUNCH.PY

### Usage
```bash
# start MPV player node and title node via launch file
ros2 launch mpvros mpv.launch.py args:="['/home/ros/Videos/bob.flv']"
```

### Arguments

> ~args (Array, default: None)\
Array with MPV command line arguments. At least one array item must be provided.

> ~title (Bool, default: true)\
Wheather to start also title node or just the MPV player node. If MPV player ends also the title node will be shutdown and the launch script ends as well. 

> ~ns (String, default: /)\
Namspace to start node(s) within.

> ~ipc_server (String: default: /tmp/mpvros-$RAND.sock)\
Override MPV IPC server socket path.

# Miscellaneous
```bash
# this ffmpeg example will create a 10 second test video, 30 fps
ffmpeg -f lavfi -i testsrc=duration=10:size=1280x720:rate=30 testsrc.mpg

# send a command from shell via topic ipc-in, the result can be retrieved 
# ia topic ipc-out
# see also https://mpv.io/manual/stable/#json-ipc
ros2 topic pub --once /ipc_in std_msgs/String "data: '{ \"command\": [\"get_property\", \"playback-time\"] }'"

# send input.conf style text-only commands, this shows the playtime on 
# the player OSD
# this will not produce a reply in the ipc-out
ros2 topic pub --once /ipc_in std_msgs/String "data: 'show-text \${playback-time}'"
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.
Please make sure to update tests as appropriate.
