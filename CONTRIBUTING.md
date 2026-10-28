# Contribuciones

MantÃ©n cada cambio centrado en un componente o un problema de integraciÃ³n verificable. Conserva las atribuciones de los componentes externos.

## Comprobaciones

Desde la raÃ­z:

```bash
python3 tools/repository_health.py
python3 -B -m unittest discover -s tests -v
```

Para cambios de interfaces, lanzadores o C++, utiliza un entorno Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
AUTONOMY_REQUIRE_ROS=1 python3 -B -m unittest discover -s tests -v
```

La comprobaciÃ³n de repositorio rechaza artefactos generados que estÃ©n versionados. Los directorios locales `build/`, `install/` y `log/` permanecen excluidos de Git y pueden existir al ejecutar las pruebas.

## Paquetes e interfaces

Declara las dependencias reales en `package.xml`. Instala los recursos utilizados en ejecuciÃ³n y comprueba su resoluciÃ³n desde `install/`. AcompaÃ±a correcciones de comportamiento con regresiones que reproduzcan el fallo.

Utiliza `--packages-up-to` para incluir dependencias propias al compilar un componente. El detector externo estÃ¡ en `external/`; verifica cualquier actualizaciÃ³n de su revisiÃ³n con una clonaciÃ³n recursiva antes de publicarla.

Escribe README y guÃ­as en espaÃ±ol claro, con requisitos, unidades, procedimientos, resultados esperados y lÃ­mites pertinentes. Documenta el funcionamiento vigente para el lector final.

## ValidaciÃ³n fÃ­sica

Indica si el cambio se comprobÃ³ con lÃ³gica aislada, servidor simulado, simulador de robot o hardware. Registra modelo, sensores, huella, lÃ­mites y parÃ¡metros al documentar resultados de movimiento. No presentes una compilaciÃ³n o un servidor Nav2 simulado como validaciÃ³n fÃ­sica.
