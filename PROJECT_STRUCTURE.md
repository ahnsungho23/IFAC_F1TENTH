# Project Structure

```text
2026_IFAC/
├── PLANNING_PIPELINE.md
├── lap_timer/
│   ├── config/
│   │   └── params.yaml
│   ├── lap_timer/
│   │   ├── __init__.py
│   │   └── lap_timer_node.py
│   ├── launch/
│   │   └── lap_timer.launch.py
│   ├── package.xml
│   ├── resource/
│   │   └── lap_timer
│   ├── rviz/
│   │   └── lap_hud.rviz
│   ├── setup.cfg
│   ├── setup.py
│   └── test/
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── monte_carlo_localization/
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── config/
│   │   └── mcl_config.yaml
│   ├── include/
│   │   └── particle_filter_cpp/
│   │       ├── particle_filter.hpp
│   │       └── utils.hpp
│   ├── launch/
│   │   └── mcl_launch.py
│   ├── maps/
│   │   ├── Spielberg_map.png
│   │   ├── Spielberg_map.yaml
│   │   ├── basement_fixed.map.yaml
│   │   ├── basement_fixed.png
│   │   ├── f2map.pgm
│   │   ├── f2map.yaml
│   │   ├── first_map.pgm
│   │   ├── first_map.yaml
│   │   ├── first_map_raceline.csv
│   │   ├── fmap.png
│   │   ├── fmap.yaml
│   │   ├── fuck_f1.pgm
│   │   ├── fuck_f1.yaml
│   │   ├── fuck_jg_1.pgm
│   │   ├── fuck_jg_1.yaml
│   │   ├── icra_2_clean.png
│   │   ├── icra_2_clean.yaml
│   │   ├── labtest.pgm
│   │   ├── labtest.yaml
│   │   ├── levine.pgm
│   │   ├── levine.yaml
│   │   ├── map_1753950572.pgm
│   │   ├── map_1753950572.yaml
│   │   ├── map_1755669035.pgm
│   │   ├── map_1755669035.yaml
│   │   ├── new_map1.png
│   │   ├── new_map1.yaml
│   │   ├── oct_25_1.pgm
│   │   ├── oct_25_1.yaml
│   │   ├── redbull_1.png
│   │   ├── redbull_1.yaml
│   │   ├── sibal1.png
│   │   ├── sibal1.yaml
│   │   ├── slam_map.png
│   │   ├── slam_map.yaml
│   │   ├── test_map.pgm
│   │   └── test_map.yaml
│   ├── package.xml
│   ├── rviz/
│   │   └── particle_filter.rviz
│   └── src/
│       ├── particle_filter.cpp
│       └── utils.cpp
├── new_map_con/
│   ├── log/
│   │   ├── COLCON_IGNORE
│   │   └── list_2025-10-30_02-10-05/
│   │       └── logger_all.log
│   ├── maps/
│   │   ├── oct28.csv
│   │   ├── oct_28.pgm
│   │   └── oct_28.yaml
│   ├── new_map_con/
│   │   ├── MAP_controller copy.py
│   │   ├── MAP_controller.py
│   │   ├── __init__.py
│   │   └── tester.py
│   ├── package.xml
│   ├── resource/
│   │   └── new_map_con
│   ├── setup.cfg
│   ├── setup.py
│   └── test/
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── planning/
│   └── global_planning/
│       ├── CMakeLists.txt
│       ├── config/
│       │   └── global_planning.yaml
│       ├── data/
│       │   └── global_waypoints.json
│       ├── include/
│       │   └── global_planning/
│       │       ├── frenet_odom_node.hpp
│       │       ├── global_planner_node.hpp
│       │       └── global_trajectory_publisher_node.hpp
│       ├── launch/
│       │   └── global_planning.launch.py
│       ├── package.xml
│       └── src/
│           ├── frenet_odom_node.cpp
│           ├── global_planner_node.cpp
│           └── global_trajectory_publisher_node.cpp
└── wpnt_publisher/
    ├── CMakeLists.txt
    ├── LICENSE
    ├── package.xml
    └── src/
        └── wpnt_publisher.cpp
```

Notes:
- Hidden/internal folders like `.git`, `.codex`, `.agents` are intentionally excluded.
