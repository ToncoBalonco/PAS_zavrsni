# ASTRO – pokretanje simulacije i A* + Potential Field Pure Pursuit navigacije


## Preduvjeti

- ROS2 Humble
- Gazebo (GZ Fortress, LTS) + `ros_gz_sim`, `ros_gz_bridge`
- `ros2_control`, `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager`, `robot_localization`,
  `twist_mux`, `xacro`
- Izgrađen i sourcean `astro` paket (`colcon build`, pa `source install/setup.bash`)

## Bitne stvari prije pokretanja (hardkodirani putevi!)

U oba launch fajla postoje **apsolutni putevi vezani za konkretnog korisnika/računalo** –
provjeri ih i po potrebi promijeni prije pokretanja:

- `sim_gazebo.launch.py` učitava svijet sa fiksne putanje:
  `/home/tona/astro_ws_pas/worlds/test_crta.world`
- `astar_pf_pp.launch.py` učitava kartu sa fiksne putanje:
  `/home/tona/astro_ws_pas/src/ASTRO/astro_slam/maps/mapa_crte_new.yaml`

Ako radiš na drugom računalu/pod drugim korisničkim imenom, ova dva reda moraš prilagoditi,
inače će launch pući na "file not found".

Također, karta (`mapa_crte_new.yaml`) mora već postojati – tj. laboratorij mora biti prethodno
mapiran (SLAM), jer `astar_pf_pp.launch.py` samo učitava gotovu kartu preko `map_server`-a, ne
radi mapiranje.



## Tipičan tok rada (sažetak)

```bash
# Terminal 1
ros2 launch astro sim_gazebo.launch.py

# pričekaj da se robot spawna (~10-15s), zatim Terminal 2
ros2 launch astro astar_pf_pp.launch.py

# u RViz-u: "2D Goal Pose" -> klik na karti -> robot planira i vozi do cilja
```


