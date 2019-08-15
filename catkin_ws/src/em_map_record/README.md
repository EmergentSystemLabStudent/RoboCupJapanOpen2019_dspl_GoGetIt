# 'em_map_record' Package

The `em_map_record` package generates a map and saves the robot position.

*   Maintainer: Yoshiki Tabuchi ([yoshiki.tabuchi@em.ci.ritsumei.ac.jp](mailto:yoshiki.tabuchi@em.ci.ritsumei.ac.jp)).
*   Author: Yoshiki Tabuchi ([yoshiki.tabuchi@em.ci.ritsumei.ac.jp](mailto:yoshiki.tabuchi@em.ci.ritsumei.ac.jp)).

**Content:**

*   [Launch Procedure](#launch-procedure)

## Launch Procedure

1.   Move the HSR robot to the origin point of the map.
2.   Push the emergency stop button and then release it.
3.   Wait until the HSR robot is in its initial pose.
4.   Start SLAM with `roslaunch em_map_record em_map_record_srv.launch new_data:=true task_name:="TASK NAME"`.
5.   In a new terminal, start the client with `roslaunch em_map_record em_map_record_cli.launch task_name:="TASK NAME"`.
