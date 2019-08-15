# 'em_door_detect' Package

The `em_door_detect` package determines whether a door is open or not.

*   Maintainer: Yoshiki Tabuchi ([yoshiki.tabuchi@em.ci.ritsumei.ac.jp](mailto:yoshiki.tabuchi@em.ci.ritsumei.ac.jp)).
*   Author: Yoshiki Tabuchi ([yoshiki.tabuchi@em.ci.ritsumei.ac.jp](mailto:yoshiki.tabuchi@em.ci.ritsumei.ac.jp)).

**Content:**

*   [Parameters](#parameters)
*   [Launch](#launch)

## Parameters

*   `door_distance`: The distance between the door and the URG sensor on the HSR.
*   `door_width`: The width of the door.

## Launch

*   Run the action server with `roslaunch em_door_open em_door_open.launch`.
*   Run the action client with `rosrun em_door_open em_door_client.py`. `em_door_client.py` is a sample code.
