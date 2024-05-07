# Tesis de A probabilistic learning-based routing scheme with path planning of autonomous agricultural vehicles for assisted harvesting tasks

## Descripción General

This work presents a combined route and path planning strategy to guide autonomous ground vehicles in scheduled
harvesting tasks within expansive crop rows subject to complex terrain conditions of agricultural fields. The proposed
planning strategy integrates i) a global planning method based on the Traveling Salesman Problem under the Capacitated
Vehicle Routing approach to prioritize harvesting positions according to criteria of minimum path length and vehicle's
load capacity, and ii) a local planning strategy based on an Informed Rapidly-exploring Random Trees (RRT$^*$) to
coordinate the previously scheduled harvesting points while simultaneously avoiding obstacles of low-traction terrain.
Moreover, the global approach aims to generate an ordered queue of harvesting locations, maximizing the harvested
product of the expected crop yield. On the other hand, in the second stage, the informed RRT$^*$ planner avoids
obstacles around the configuration space of the robotic vehicle and incorporates a dynamical model of the vehicle motion
dynamics to meet constraints compatible with those of the planned path. Experimental results demonstrated the
effectiveness of the proposed planning strategy, achieving smooth and collision-free paths, making it suitable for
practical applications in precision agriculture

## Requisitos Previos

Asegúrate de tener instalado lo siguiente antes de comenzar:

- CoppeliaSim (Versión 4.5.1)
- Python (Versión 3.9)

## Instalación

1. **CoppeliaSim:**
    - Descarga CoppeliaSim desde [el sitio oficial](https://www.coppeliarobotics.com/downloads).
    - Sigue las instrucciones de instalación proporcionadas en la documentación.

2. **Python y Dependencias:**
    - Instala Python > 3.9 desde [python.org](https://www.python.org/downloads/).
    - Instala las dependencias requeridas ejecutando el siguiente comando:
      ```bash
      pip install -r requirements.txt
      ```

## Obtención del Mapa de CoppeliaSim

1. Abre CoppeliaSim
1. Abre el archivo de la carpeta coppelia_map

## Consideraciones

- El mapa debe tener un plano de trabajo de 1x15 metros
- Si deseas ejecutar la aplicación On Line obteniendo el mapa desde coppelia, debes cambiar en el archivo config.py el
  valor de la variable ON_LINE a True
- Si deseas ejecutar la aplicación Off Line obteniendo el mapa desde un archivo, debes cambiar en el archivo config.py
  el valor de la variable ON_LINE a False

## Ejecución del Programa

1. Utiliza el script `app.py` de la carpeta steps para ejecutar el programa
    ```bash
    python app.py
    ```

## Ejecución por partes

1. Utiliza el script `step_1_get_data.py` de la carpeta steps para obtener el mapa procesado desde coppelia
    ```bash
    python step_1_get_data.py

2. Utiliza el script `step_2_tsp.py` de la carpeta steps para ejecutar el algoritmo de TSP
    ```bash
    python step_2_tsp.py

3. Utiliza el script `step_3_rrt.py` de la carpeta steps para ejecutar el algortimo de RRT
    ```bash
    python step_3_rrt.py
