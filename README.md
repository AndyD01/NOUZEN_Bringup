# nouzen_bringup

![ROS2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Platform](https://img.shields.io/badge/Platform-NOUZEN%204WD-red)
![Build](https://img.shields.io/badge/Build-ament__cmake-yellow)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

Pachet ROS 2 de pornire (*bringup*) pentru robotul mobil autonom **NOUZEN**. Conține descrierea URDF/xacro, interfața hardware `ros2_control`, fișierele de configurare pentru drivere și senzori, precum și fișierele de lansare care aduc robotul într-o stare operațională, gata să primească comenzi de la stiva de navigație.

## Rol în arhitectura NOUZEN

`nouzen_bringup` este **primul pachet pornit** pe robot și asigură:

1. **Activarea drivetrain-ului** prin `ros2_control` și pornirea controlerelor (`diff_drive_controller`, `joint_state_broadcaster`)
2. **Pornirea senzorilor de bord**: LiDAR LD19, cameră USB pentru detecția marker-elor AprilTag
3. **Publicarea modelului robotului** (URDF, arborele TF static)
4. **Publicarea pozelor de andocare** prin scriptul `dock_pose_publisher`, care face puntea dintre `apriltag_ros` și `opennav_docking`

Peste acest strat se montează ulterior SLAM Toolbox / AMCL, Nav2, `opennav_docking` și nivelul de orchestrare a misiunilor intralogistice.

## Structura pachetului

```
nouzen_bringup/
├── bringup/
│   ├── config/
│   │   ├── camera_info.yaml         # calibrare intrinsecă cameră 640x480
│   │   ├── ld19.yaml                # configurație driver LiDAR LD19
│   │   ├── nouzen_controllers.yaml  # parametri ros2_control + diff_drive_controller
│   │   └── teleop_xbox.yaml         # mapare butoane controller Xbox
│   └── launch/
│       ├── include/
│       │   ├── camera.launch.py     # v4l2_camera node, YUYV 640x480 @ 15 fps
│       │   └── ld19.launch.py       # driver ldlidar_stl_ros2
│       ├── nouzen_base.launch.py    # robot_state_publisher + controller_manager + spawners
│       ├── robot.launch.py          # launch unificat: bază + senzori + dock_pose_publisher
│       ├── sensors.launch.py        # cameră + LiDAR
│       └── teleop_xbox.launch.py    # joy + teleop_twist_joy
├── description/
│   └── urdf/
│       ├── nouzen.urdf.xacro            # punct de intrare URDF
│       ├── nouzen_description.xacro     # link-uri și joint-uri
│       ├── nouzen_materials.xacro       # materiale și culori
│       └── nouzen_ros2_control.xacro    # tag-uri <ros2_control> și parametri PID
├── hardware/
│   ├── include/
│   │   └── nouzen_hardware_interface.hpp
│   └── src/
│       └── nouzen_hardware_interface.cpp  # SystemInterface, comunicație serială cu ESP32
├── scripts/
│   └── dock_pose_publisher.py       # republicare pose AprilTag pe /detected_dock_pose
├── CMakeLists.txt
├── package.xml
├── plugins.xml                      # export pluginlib pentru hardware interface
├── LICENSE
└── README.md
```

## Interfața hardware (`nouzen_hardware_interface`)

Implementează `hardware_interface::SystemInterface` și face legătura dintre `ros2_control` și firmware-ul ESP32 (vezi [ROS2_ESP32_Bridge](https://github.com/AndyD01/ROS2_ESP32_Bridge)). Este exportată ca plugin prin `plugins.xml` și încărcată la runtime de către `controller_manager` pe baza tag-urilor `<ros2_control>` din URDF.

Funcționalități principale:
- Citește parametrii PID (`Kp`, `Ki`, `Kd`, `Ko`) din URDF și îi transmite la ESP32 prin comanda `u`
- Convertește comenzile de viteză `joint_velocity` în vitezele țintă pe motor (comanda `m M1 M2 M3 M4`)
- Citește contoarele de encoder (comanda `e`) și actualizează stările articulațiilor
- Aplică oprire automată dacă fluxul de comenzi se întrerupe

## Senzori și drivere

| Senzor | Driver / Pachet | Port | Configurare |
|--------|-----------------|------|-------------|
| LiDAR LD19 | `ldlidar_stl_ros2` | `/dev/ttyUSB1` | `bringup/config/ld19.yaml` |
| Cameră USB | `v4l2_camera` | `/dev/video0` | YUYV, 640x480, `time_per_frame=[1,15]`, `mmap` |
| Driver motoare | `ros2_control` + ESP32 bridge | `/dev/ttyUSB0` | parametri PID din URDF |

> **Observație importantă pentru cameră**: pe ROS 2 Jazzy, `v4l2_camera` **nu poate decoda MJPG intern**. Trebuie folosit formatul `YUYV`, altfel pipeline-ul de detecție AprilTag nu primește imagini. Cu această configurație se obține un debit stabil de aproximativ 18 FPS chiar cu `apriltag_ros` activ.

## Configurare controlere (`nouzen_controllers.yaml`)

Două controlere se pornesc automat la lansare:

| Controler | Tip | Rol |
|-----------|-----|-----|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publică `/joint_states` |
| `diff_drive_controller` | `diff_drive_controller/DiffDriveController` | Convertește `cmd_vel` în comenzi pe roți |

Parametri cinematici ai robotului:

- `wheel_separation` = **0.410 m**
- `wheel_radius` = **0.0375 m**
- Topic comandă: `/diff_drive_controller/cmd_vel` (sau remapat la `/cmd_vel` în launch)

## `dock_pose_publisher.py`

Script Python care intermediază între `apriltag_ros` și `opennav_docking`:

1. Se abonează la `/detections` (mesaje `AprilTagDetectionArray`)
2. Filtrează detecția în funcție de ID-ul tag-ului dock-ului activ
3. Publică un `PoseStamped` pe `/detected_dock_pose`, consumat de `opennav_docking` ca *external detection*
4. Permite schimbarea ID-ului tag-ului țintă la runtime prin **`SetParameters`**, înainte de fiecare acțiune de andocare

ID-urile celor 5 stații de andocare folosesc familia `tag36h11` cu dimensiunea fizică de 150 mm (ID-uri 1 - 5).

## Cum se compilează

Repo-ul se clonează în `src/` al unui workspace ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndyD01/nouzen_bringup.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nouzen_bringup --symlink-install
source install/setup.bash
```

## Cum se pornește

### Lansare completă (bază + senzori + dock pose publisher)

```bash
ros2 launch nouzen_bringup robot.launch.py
```

### Doar baza mobilă (fără senzori)

```bash
ros2 launch nouzen_bringup nouzen_base.launch.py
```

### Doar senzorii (LiDAR + cameră)

```bash
ros2 launch nouzen_bringup sensors.launch.py
```

### Teleoperare manuală cu controller Xbox

```bash
ros2 launch nouzen_bringup teleop_xbox.launch.py
```

## Configurația de rețea

Robotul rulează cu `ROS_DOMAIN_ID=28` pentru a izola comunicația în rețeaua locală. Setarea trebuie făcută atât pe Raspberry Pi cât și pe stația de dezvoltare:

```bash
export ROS_DOMAIN_ID=28
```

## Dependențe principale

| Pachet | Sursă |
|--------|-------|
| `ros2_control`, `ros2_controllers` | `apt: ros-jazzy-ros2-control*` |
| `diff_drive_controller` | `apt: ros-jazzy-diff-drive-controller` |
| `v4l2_camera` | `apt: ros-jazzy-v4l2-camera` |
| `apriltag_ros` | `apt: ros-jazzy-apriltag-ros` |
| `ldlidar_stl_ros2` | sursă, [Ldrobot oficial](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) |
| `xacro`, `robot_state_publisher` | `apt: ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher` |

Toate dependențele declarate în `package.xml` se instalează automat cu `rosdep install`.

## Licență

Distribuit sub licența **Apache License 2.0**. Vezi fișierul [LICENSE](LICENSE) pentru detalii complete.

## Context academic

Proiect dezvoltat în cadrul lucrării de licență privind un sistem autonom de navigație și andocare pentru robot mobil diferențial în scenarii intralogistice, la **Facultatea de Inginerie Industrială și Robotică (FIIR), Universitatea Politehnica din București**, specializarea **Informatică Aplicată în Inginerie Industrială (IAII)**, sub coordonarea **Conf. Dr. Ing. Bogdan-Felician Abaza**.
