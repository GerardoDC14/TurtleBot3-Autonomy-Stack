# Arquitectura e interfaces

## Paquetes

Los paquetes propios estÃ¡n en `src/`; el cÃ³digo externo se conserva en `external/` y queda fuera de la bÃºsqueda de colcon.

```mermaid
flowchart LR
  L[LiDAR y odometrÃ­a] --> S[SLAM Toolbox o AMCL]
  S --> M[Mapa y TF]
  M --> E[Explorador Python o C++]
  P[Administrador de POIs] --> N[Nav2]
  E --> N
  N --> B[Base TurtleBot3]
  C[Detector externo] --> H[Anotador HAZMAT]
  M --> H
  H --> R[RViz]
  U[Entrada del operador] --> A[Referencias acotadas por URDF]
  A --> J[VisualizaciÃ³n de articulaciones]
```

## NavegaciÃ³n

`mission_launch.py` selecciona SLAM o localizaciÃ³n con mapa guardado mediante Nav2 bringup. La fuente global de mapa y TF depende de ese modo. El explorador puede ser Python, C++ o ninguno.

El explorador Python y el administrador de POIs comparten `autonomy_common.NavigationTask`. Este componente envÃ­a solicitudes asÃ­ncronas, procesa aceptaciÃ³n y resultados, y solicita cancelaciÃ³n al vencer el tiempo lÃ­mite. Mantiene la propiedad del objetivo hasta recibir un resultado terminal. El explorador C++ implementa el mismo contrato dentro de su ejecutor de un solo hilo.

Solo un cliente de misiÃ³n debe estar activo. Este contrato evita superposiciones dentro de cada cliente; no impide que un operador o un cliente externo sustituya un objetivo en Nav2.

Las fronteras se agrupan por conectividad de ocho vecinos. El destino es una celda libre del grupo prÃ³xima a su centro, con margen de obstÃ¡culos conocidos. Se consideran el origen rotado del OccupancyGrid, los lÃ­mites de reintentos y las visitas confirmadas por Nav2. La utilidad resta una penalizaciÃ³n de distancia al Ã¡rea desconocida; no representa un coste de trayectoria calculado por el planificador.

## Datos

| Interfaz | Tipo | Contrato |
|---|---|---|
| `map` | `nav_msgs/OccupancyGrid` | Mapa planar vÃ¡lido en el marco global; suscripciÃ³n transitoria local |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 activo y configurado para la base |
| `scan` â†’ `adjusted_scan` | `sensor_msgs/LaserScan` | GeometrÃ­a coherente, QoS de sensores y reducciÃ³n opcional de rayos |
| `hazmat_publisher` | `std_msgs/String` | Etiqueta recibida; `No detections` no genera observaciÃ³n |
| `odom` | `nav_msgs/Odometry` | Pose sellada en `header.frame_id`, transformable al mapa |
| `hazmat_markers` | `visualization_msgs/MarkerArray` | Conjunto acotado de ubicaciones de observaciÃ³n |
| `motor1_position`, `motor2_position` | `std_msgs/Float32` | Referencias finitas en radianes, limitadas por URDF |
| `joint_states` | `sensor_msgs/JointState` | RepresentaciÃ³n de referencias; no realimentaciÃ³n fÃ­sica |

Los nombres son relativos y admiten remapeo ROS. `map_frame` y `robot_frame` se configuran mediante parÃ¡metros.

## Responsabilidades fÃ­sicas

Nav2 controla trayectorias y recuperaciÃ³n de la base. Los nodos de misiÃ³n no sustituyen el paro fÃ­sico. El mecanismo necesita un controlador de actuadores independiente. La anotaciÃ³n HAZMAT registra la posiciÃ³n del robot al recibir una etiqueta; localizar el objeto requiere informaciÃ³n geomÃ©trica adicional del sistema de percepciÃ³n.
