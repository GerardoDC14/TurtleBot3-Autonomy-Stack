# GuÃ­a de operaciÃ³n

## 1. Equipo y entorno

Utiliza Ubuntu 22.04 con ROS 2 Humble. Instala rosdep, colcon, los paquetes TurtleBot3, Nav2 y SLAM Toolbox. La resoluciÃ³n de dependencias del README instala los requisitos declarados de los paquetes propios, incluidos NumPy, SciPy, PyYAML y Tk.

Carga `/opt/ros/humble/setup.bash` e `install/setup.bash` en cada terminal. En equipos distribuidos, utiliza el mismo `ROS_DOMAIN_ID` y permite el descubrimiento ROS 2 en la red. Selecciona `TURTLEBOT3_MODEL` segÃºn el robot: `burger`, `waffle` o `waffle_pi`.

Antes de iniciar autonomÃ­a, verifica:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

La base debe tener alimentaciÃ³n, odometrÃ­a y TF. Comprueba la huella y los lÃ­mites de velocidad en los parÃ¡metros Nav2 del modelo. MantÃ©n disponible el paro fÃ­sico del robot.

## 2. CompilaciÃ³n

Desde la raÃ­z del repositorio:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

Para compilar un componente con sus dependencias internas, utiliza `--packages-up-to`; por ejemplo:

```bash
colcon build --symlink-install --packages-up-to turtlebot3_explore
```

El detector externo estÃ¡ fuera de `src/` y no forma parte de este perfil.

## 3. Mapeo y localizaciÃ³n

Inicia primero el robot o el simulador. El lanzador de misiÃ³n acepta:

| Argumento | Valores y funciÃ³n |
|---|---|
| `mode` | `slam` para generar mapa; `localization` para usar un mapa guardado |
| `model` | Modelo TurtleBot3; toma `TURTLEBOT3_MODEL` o `burger` |
| `explorer` | `none` por defecto; `python` o `cpp` para explorar automÃ¡ticamente |
| `map` | Archivo YAML del mapa; obligatorio en localizaciÃ³n |
| `nav2_params` | YAML Nav2 opcional; por defecto usa el del modelo en Humble |
| `explore_params` | YAML del explorador; por defecto usa `explore_params.yaml` |
| `use_sim_time` | `true` con reloj del simulador; `false` con tiempo real |

Mapeo sin movimiento autÃ³nomo:

```bash
ros2 launch turtlebot3_explore mission_launch.py \
  mode:=slam model:=burger explorer:=none use_sim_time:=false
```

Espera un mapa coherente y una transformaciÃ³n `map -> odom -> base_link`. En otra terminal puedes iniciar RViz con la configuraciÃ³n de TurtleBot3:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz"
```

Para guardar el mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/mi_mapa"
```

Para localizar, detÃ©n el mapeo e inicia `mode:=localization map:=/ruta/mi_mapa.yaml`. Establece la pose inicial en RViz y espera a que coincidan el escaneo y el mapa. No ejecutes otro SLAM o AMCL sobre la misma transformaciÃ³n global.

## 4. ExploraciÃ³n

El explorador Python es la opciÃ³n predeterminada. Para ejecutarlo con Nav2 y el mapa activos:

```bash
ros2 launch turtlebot3_explore explore_launch.py use_sim_time:=false
```

La alternativa C++ se ejecuta con `ros2 run explore_cpp explore_node`. Ambos pueden recibir el mismo YAML con `--ros-args --params-file /ruta/explore_params.yaml`.

Cada explorador selecciona celdas libres prÃ³ximas al centro de grupos de frontera. La selecciÃ³n considera Ã¡rea desconocida y distancia; Nav2 decide si existe una trayectoria vÃ¡lida. Los mapas deben contener ocupaciÃ³n `-1` para desconocido y `0..100` para probabilidad de ocupaciÃ³n, con un origen planar vÃ¡lido.

| ParÃ¡metro | Predeterminado | Unidad o significado |
|---|---:|---|
| `info_radius` | 1.0 | Semiancho de ventana para contar Ã¡rea desconocida, m |
| `frontier_visit_threshold` | 0.3 | Radio de exclusiÃ³n de objetivos alcanzados, m |
| `max_attempts` | 3 | MÃ¡ximo de envÃ­os por coordenada de frontera |
| `timer_interval` | 2.0 | Intervalo de selecciÃ³n, s |
| `min_frontier_size` | 5 | MÃ­nimo de celdas conectadas por frontera |
| `minimum_goal_distance` | 0.3 | Distancia mÃ­nima desde el robot, m |
| `clearance_radius` | 0.2 | SeparaciÃ³n de obstÃ¡culos conocidos y bordes, m |
| `free_threshold` | 25 | Mayor valor de ocupaciÃ³n considerado libre |
| `distance_weight` | 0.5 | PenalizaciÃ³n de distancia en la utilidad, mÂ²/m |
| `navigation_timeout` | 120.0 | Tiempo de pared antes de solicitar cancelaciÃ³n, s |
| `retry_cooldown` | 10.0 | Espera tras fallo o cancelaciÃ³n, s |
| `map_frame` / `robot_frame` | `map` / `base_link` | Marcos TF |

Ajusta la separaciÃ³n a la huella fÃ­sica del robot. La comprobaciÃ³n de una celda no sustituye los mapas de costes, la huella ni la comprobaciÃ³n de trayectorias de Nav2.

Un rechazo libera el objetivo para un reintento acotado. Un tiempo lÃ­mite solicita cancelaciÃ³n; no se envÃ­a otro objetivo hasta recibir un resultado terminal. Si la comunicaciÃ³n se pierde, la misiÃ³n conserva el objetivo y permanece en espera.

Al pulsar Ctrl+C, el explorador solicita cancelar su acciÃ³n y espera hasta tres segundos. Si no se confirma la cancelaciÃ³n, comprueba y detÃ©n la base. Los tiempos lÃ­mite usan tiempo de pared incluso si el simulador estÃ¡ pausado. Ejecuta un solo cliente de misiÃ³n a la vez; Nav2 puede aceptar objetivos de otros clientes y sustituir el objetivo activo.

## 5. Puntos de interÃ©s

Con localizaciÃ³n estable y sin un explorador activo:

```bash
ros2 run turtlebot3_poi_navigation poi_manager
```

AÃ±ade un nombre, coordenadas X/Y en metros del marco global y un radio de visualizaciÃ³n positivo en metros. El radio controla el marcador; las tolerancias de llegada se configuran en Nav2. Los destinos se guardan de forma atÃ³mica en `~/.ros/turtlebot3_autonomy/pois.yaml`. Puedes elegir otra ruta con `-p poi_file:=/ruta/pois.yaml`.

La interfaz permite un objetivo a la vez, muestra el resultado y ofrece Cancel. Cerrar la ventana solicita cancelaciÃ³n. No cargues POIs de otro mapa sin comprobar sus coordenadas.

## 6. Escaneo

```bash
ros2 run laser_scan_adjuster adjust_scan
```

El adaptador recibe `scan` y publica `adjusted_scan` con QoS de sensores. Por defecto conserva todas las mediciones. Para limitar la densidad:

```bash
ros2 run laser_scan_adjuster adjust_scan --ros-args -p max_samples:=227
```

La reducciÃ³n selecciona rayos existentes con un paso entero y produce como mÃ¡ximo el lÃ­mite solicitado; no garantiza exactamente esa cantidad. Actualiza el incremento angular, el Ã¡ngulo final, el tiempo entre rayos y las intensidades. Nunca rellena escaneos cortos. Descarta entradas vacÃ­as o con geometrÃ­a inconsistente.

Configura explÃ­citamente el tÃ³pico del consumidor si debe usar `adjusted_scan`; el adaptador no modifica SLAM ni Nav2.

## 7. Observaciones HAZMAT

```bash
ros2 run hazmat_marker hazmat_marker
```

El nodo recibe nombres en `hazmat_publisher` como `std_msgs/String` y odometrÃ­a en `odom`. Transforma la pose de odometrÃ­a a `map` usando su sello temporal. Descarta observaciones con una pose de mÃ¡s de `max_pose_age:=0.5` segundos o sin TF disponible.

Los marcadores indican dÃ³nde estaba el robot al recibir el nombre. El mensaje String no incluye el tiempo de adquisiciÃ³n de la imagen, profundidad ni posiciÃ³n del objeto: la latencia de inferencia limita la asociaciÃ³n temporal. No interpretes el marcador como la ubicaciÃ³n exacta del material.

El nodo agrupa observaciones de la misma etiqueta dentro de `min_separation:=0.5` m y conserva hasta `max_observations:=200`. Publica el conjunto en `hazmat_markers` con durabilidad transitoria local; configura RViz con esa durabilidad para recibir el estado al conectarse.

Para probar la anotaciÃ³n con el robot localizado:

```bash
ros2 topic pub --once /hazmat_publisher std_msgs/msg/String "{data: flammable}"
```

La inferencia es un componente externo. InicialÃ­zalo con `git submodule update --init --recursive` y consulta `external/HAZMAT_Detection/README.md` y [DeepHAZMAT](https://github.com/mrl-amrl/DeepHAZMAT). El servidor externo requiere rutas locales del mÃ³dulo y de los archivos del modelo; su cliente utiliza cÃ¡mara comprimida y un servidor TCP en `127.0.0.1:10000`. Configura esas rutas y dependencias antes de ejecutarlo. La compilaciÃ³n y las pruebas de este workspace no validan ese servidor ni sus pesos.

## 8. Mecanismo

```bash
ros2 run motor_position_controller motor_position_controller
ros2 run joint_state_publisher_custom joint_state_publisher
```

El publicador lee los lÃ­mites de `joint_motor_1_rotor` y `joint_motor_2_rotor` del URDF instalado. En el modelo suministrado son Â±3.14 rad por eje. Puedes proporcionar un URDF fÃ­sico diferente con `model_file` o el parÃ¡metro `robot_description`.

Comandos: `T 1.0` para ambos ejes, `T1 -0.5` o `T2 0.5` para uno, y `q` para cerrar. Las referencias se publican a 20 Hz y se acotan por eje; se rechazan NaN e infinitos. Un eje no recibe comandos hasta su primera entrada vÃ¡lida.

`joint_state_publisher_custom` representa referencias comandadas a 50 Hz. No mide encoders. El controlador fÃ­sico debe implementar sus propios lÃ­mites, realimentaciÃ³n, vigilancia de comunicaciÃ³n y paro. Dejar de publicar no garantiza que un actuador se detenga.

## 9. ValidaciÃ³n

Ejecuta los comandos de pruebas del README. Las pruebas sin ROS comprueban lÃ³gica y callbacks con mensajes de prueba. Las pruebas ROS compilan e inician los ejecutables contra un servidor de acciones simulado, sin publicar velocidades de la base. Para validar movimiento y percepciÃ³n, utiliza primero simulaciÃ³n y despuÃ©s pruebas fÃ­sicas supervisadas con la configuraciÃ³n real.
