# TurtleBot3 Autonomy Stack

Workspace de ROS 2 Humble para explorar con TurtleBot3, navegar a puntos de interÃ©s, registrar observaciones HAZMAT y visualizar un mecanismo de dos ejes.

El sistema delega la planificaciÃ³n y el movimiento de la base a Nav2. Las herramientas de mecanismo publican referencias de posiciÃ³n; requieren un controlador fÃ­sico independiente.

## Requisitos

- Ubuntu 22.04 con ROS 2 Humble.
- TurtleBot3 y sus paquetes de navegaciÃ³n, Nav2 y SLAM Toolbox.
- Python 3, Git, rosdep y colcon.
- Red ROS 2 configurada entre el robot y el equipo de operaciÃ³n.
- Interfaz grÃ¡fica para administrar puntos de interÃ©s con Tk.

## PreparaciÃ³n

```bash
git clone --recurse-submodules https://github.com/GerardoDC14/TurtleBot3-Autonomy-Stack.git
cd TurtleBot3-Autonomy-Stack
source /opt/ros/humble/setup.bash

# Ejecuta rosdep init una vez por equipo, si todavÃ­a no estÃ¡ configurado.
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

colcon build --symlink-install
source install/setup.bash
```

Los ocho paquetes de `src/` forman el workspace principal. El detector externo se conserva en `external/HAZMAT_Detection`, con una revisiÃ³n fija y sus propios requisitos; `external/COLCON_IGNORE` lo excluye de la compilaciÃ³n predeterminada.

## Primera misiÃ³n

Inicia primero la base o el simulador y comprueba que publica `/scan`, `/odom` y TF. Selecciona el modelo fÃ­sico correcto:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_explore mission_launch.py \
  mode:=slam model:=burger explorer:=none use_sim_time:=false
```

Este comando inicia SLAM y Nav2 sin enviar objetivos de exploraciÃ³n. Comprueba el mapa, la posiciÃ³n y la huella del robot en RViz. Para iniciar la exploraciÃ³n en otra terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot3_explore explore_launch.py use_sim_time:=false
```

TambiÃ©n puedes iniciar una misiÃ³n completa con `explorer:=python` o `explorer:=cpp`. Usa un solo explorador o administrador de misiones a la vez. En simulaciÃ³n, establece `use_sim_time:=true` en todos los nodos.

Para navegar sobre un mapa guardado, detÃ©n la misiÃ³n de SLAM e inicia:

```bash
ros2 launch turtlebot3_explore mission_launch.py \
  mode:=localization model:=burger explorer:=none \
  map:="$PWD/maps/mapLabB.yaml" use_sim_time:=false
```

En este modo, establece la pose inicial en RViz antes de enviar objetivos. El lanzador selecciona SLAM o localizaciÃ³n con AMCL; cada modo tiene una Ãºnica fuente del mapa y de la transformaciÃ³n global.

## Componentes

| Paquete | FunciÃ³n |
|---|---|
| `autonomy_common` | GestiÃ³n asÃ­ncrona de un objetivo Nav2, cancelaciÃ³n y tiempos lÃ­mite |
| `turtlebot3_explore` | Explorador predeterminado en Python |
| `explore_cpp` | Explorador alternativo en C++17 |
| `turtlebot3_poi_navigation` | Interfaz de destinos nombrados y almacenamiento YAML |
| `laser_scan_adjuster` | ValidaciÃ³n y reducciÃ³n opcional de muestras LaserScan |
| `hazmat_marker` | Ubicaciones donde el robot recibiÃ³ una detecciÃ³n HAZMAT |
| `motor_position_controller` | Referencias de posiciÃ³n acotadas por el URDF |
| `joint_state_publisher_custom` | VisualizaciÃ³n de posiciones comandadas del mecanismo |

## Comprobaciones

```bash
python3 tools/repository_health.py
python3 -B -m unittest discover -s tests -v
```

Las pruebas locales de lÃ³gica requieren NumPy, SciPy y PyYAML. Las pruebas ROS requieren compilar y cargar el workspace. Para exigir que se ejecuten, sin omitirlas por falta de ROS:

```bash
AUTONOMY_REQUIRE_ROS=1 python3 -B -m unittest discover -s tests -v
```

GitHub Actions comprueba el repositorio, compila los paquetes en Humble y ejecuta las pruebas con un servidor Nav2 simulado. Estas comprobaciones no validan sensores, movimiento real, precisiÃ³n del detector ni lÃ­mites mecÃ¡nicos fÃ­sicos.

## GuÃ­as

- [OperaciÃ³n, parÃ¡metros y precauciones](docs/OPERATIONS.md).
- [Arquitectura e interfaces](docs/ARCHITECTURE.md).
- [Alcance de validaciÃ³n y limitaciones](docs/PROJECT_STATUS.md).
- [Contribuciones y pruebas](CONTRIBUTING.md).
