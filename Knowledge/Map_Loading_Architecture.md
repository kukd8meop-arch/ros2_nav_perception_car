# 地图加载架构说明

## 地图文件位置

地图存放在 `slam` 包的 `maps/` 目录下：

```
src/slam/maps/
├── map_01.yaml    ← 地图元数据
└── map_01.pgm     ← 栅格地图图片（黑白占用栅格）
```

`map_01.yaml` 内容示例：

```yaml
image: map_01.pgm      # 对应的图片文件名
mode: trinary
resolution: 0.05       # 每像素代表 0.05 米
origin: [-9.19, -10.1, 0]  # 地图原点（米），对应图片左下角在世界坐标中的位置
negate: 0
occupied_thresh: 0.65  # 像素值高于此阈值视为障碍物
free_thresh: 0.25      # 像素值低于此阈值视为自由空间
```

---

## 调用链

```
bash sim_nav.sh [map:=map_01]
  │
  └─► ros2 launch navigation sim_nav_full.launch.py map:=map_01
        │
        │  src/navigation/launch/sim_nav_full.launch.py  line 202
        │  slam_pkg = get_package_share_directory('slam')
        │  map_yaml = os.path.join(slam_pkg, 'maps', map_name + '.yaml')
        │             → install/slam/share/slam/maps/map_01.yaml
        │  （若文件不存在直接抛出 FileNotFoundError，给出友好提示）
        │
        ├─► nav2_map_server（map_server 节点）
        │     读取 map_01.yaml + map_01.pgm
        │     发布话题：/map（nav_msgs/msg/OccupancyGrid）
        │
        ├─► AMCL
        │     订阅 /map + /scan_raw
        │     基于粒子滤波在地图上定位机器人
        │     发布：/tf（map → odom）+ /amcl_pose
        │
        └─► Nav2 Costmap（全局 + 局部）
              订阅 /map + /scan_raw
              在地图基础上构建代价地图，用于路径规划
```

---

## 默认值来源

| 文件 | 行号 | 默认值 |
|------|------|--------|
| `sim_nav_full.launch.py` | 约 185 行 `DeclareLaunchArgument('map', default_value='map_01')` | `map_01` |
| `navigation.launch.py` | 23 行 `LaunchConfiguration('map', default='map_01')` | `map_01` |
| `navigation_sim.launch.py` | 217 行 `DeclareLaunchArgument('map', default_value='map_01')` | `map_01` |
| `bringup.launch.py` | 55 行 `os.path.join(..., 'maps', 'map_01.yaml')` | `map_01` |

---

## 如何切换地图

**方法 1：命令行参数（推荐）**

```bash
bash sim_nav.sh map:=map_02
# 或
ros2 launch navigation sim_nav_full.launch.py map:=map_02
```

**方法 2：新建地图文件**

1. 将建图结果保存到 `src/slam/maps/`：
   ```bash
   ros2 run nav2_map_server map_saver_cli -f src/slam/maps/map_02
   ```
   生成 `map_02.yaml` 和 `map_02.pgm`。

2. 重新构建（使文件被安装到 install/）：
   ```bash
   colcon build --packages-select slam --symlink-install
   ```

3. 启动时指定：
   ```bash
   bash sim_nav.sh map:=map_02
   ```

---

## 地图文件安装路径

`setup.py` 中的配置将 `maps/` 目录下所有文件安装到 share：

```python
# src/slam/setup.py line 19
(os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
```

运行时实际读取的是安装后的路径：

```
install/slam/share/slam/maps/map_01.yaml
install/slam/share/slam/maps/map_01.pgm
```

使用 `--symlink-install` 构建时为符号链接，修改 `src/slam/maps/` 下的文件无需重新 build 即可生效。
