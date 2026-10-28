# Estado y alcance de validaciÃ³n

El workspace es un prototipo de autonomÃ­a para ROS 2 Humble y TurtleBot3. Su configuraciÃ³n de referencia utiliza Ubuntu 22.04.

| Componente | ValidaciÃ³n automatizada prevista |
|---|---|
| Metadatos y sintaxis | ComprobaciÃ³n independiente de ROS |
| NavegaciÃ³n asÃ­ncrona | Resultados, rechazo, cancelaciÃ³n, tiempos lÃ­mite y aceptaciÃ³n tardÃ­a |
| Fronteras y mapa | RotaciÃ³n del origen, lÃ­mites, agrupaciÃ³n y reintentos |
| Sensores y visualizaciÃ³n | Escaneos, transformaciones HAZMAT y posiciones POI |
| Mecanismo | Lectura del URDF, lÃ­mites y rechazo de valores no finitos |
| Workspace Humble | ResoluciÃ³n de dependencias, compilaciÃ³n e importaciÃ³n |
| Ejecutables Python y C++ | Servidor Nav2 simulado: Ã©xito, rechazo, aborto, tiempo lÃ­mite y cierre |

El resultado aplicable a una revisiÃ³n es el de su ejecuciÃ³n de GitHub Actions. Las pruebas sin ROS omiten explÃ­citamente la integraciÃ³n; `AUTONOMY_REQUIRE_ROS=1` convierte la falta de ROS en error.

## Limitaciones operativas

- La geometrÃ­a, huella, velocidad y lÃ­mites deben corresponder al robot fÃ­sico.
- Un objetivo libre en el mapa puede ser inalcanzable; Nav2 determina la trayectoria y el resultado.
- Si se pierde comunicaciÃ³n durante una acciÃ³n, no se confirma la cancelaciÃ³n ni se garantiza el paro.
- Los archivos POI pertenecen al marco y al mapa utilizados al crearlos.
- Los marcadores HAZMAT describen ubicaciones de observaciÃ³n; no estiman la posiciÃ³n tridimensional del objeto ni certifican la detecciÃ³n.
- El detector externo requiere cÃ¡mara, DeepHAZMAT, rutas de instalaciÃ³n y archivos de modelo. Su inferencia no estÃ¡ cubierta por las pruebas del workspace.
- La visualizaciÃ³n del mecanismo utiliza referencias comandadas. Para control fÃ­sico se necesitan encoders, controlador y protecciÃ³n independientes.

## Componentes externos

`external/HAZMAT_Detection` fija una revisiÃ³n del repositorio de alonsorc9. Conserva su cÃ³digo, atribuciÃ³n y condiciones originales. Su presencia no aÃ±ade dependencias de inferencia al workspace principal.
