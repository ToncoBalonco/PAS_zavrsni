## ASTRO Navigation

1. Running ASTRO robot state publisher \
    Make sure you have source your **astro_ws**
    ```bash
    ros2 launch astro rsp.launch.py
    ```
2. Running mapping method \
    Make sure you have source your **astro_ws**
    ```bash
    ros2 launch astro_navigation slam_toolbox_online_async.launch.py
    ```
    Drive robot arround to create map
3. Saving map \
   You can save it from _rviz_ or _Terminal_
    ```bash
    ros2 run nav2_map_server map_saver_cli -f moja_mapa
    ```
5. Starting loclization of the robot
   ```bash
    ros2 launch astro_navigation localization.launch.py map:=moja_mapa.yaml
    ```
6. Starting navigation of the robot \
   Make sure you have source your **astro_ws**
   ```bash
    ros2 launch astro_navigation navigation.launch.py
    ```
7. Starting _rviz_
    ```bash
    ros2 run rviz2 rviz2 -d ~/astro_ws/src/ASTRO/astro_navigation/rviz/nav2_default_view.rviz
    ```
