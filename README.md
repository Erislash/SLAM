# Graph SLAM con GTSAM

Trabajo práctico de Robótica Móvil. Optimización de pose graphs en 2D y 3D con GTSAM desde Python, con una solución batch (Gauss-Newton) y una incremental (ISAM2).

## Estructura

```
.
├── data/
│   ├── input_INTEL_g2o.g2o      # dataset 2D de ejemplo
│   └── parking-garage.g2o       # dataset 3D de ejemplo
├── reader.py                    # lectura de archivos G2O
├── batch_2d.py                  # 2D: solución batch con Gauss-Newton
├── incremental_2d.py            # 2D: solución incremental con ISAM2
├── batch_3d.py                  # 3D: solución batch con Gauss-Newton
├── incremental_3d.py            # 3D: solución incremental con ISAM2
├── requirements.txt
└── README.md
```

## Requisitos

- Python 3.10 o superior
- Las dependencias de `requirements.txt`: `gtsam`, `numpy` y `matplotlib`

`numpy` tiene que ser una versión menor a 2: con `gtsam` 4.2 y `numpy` 2 el programa termina con un _seg fault_.

## Instalación

Desde la carpeta del repositorio:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Ejecución

Con el entorno activado:

| Parte | Comando |
| --- | --- |
| 2D, batch (Gauss-Newton) | `python3 batch_2d.py` |
| 2D, incremental (ISAM2) | `python3 incremental_2d.py` |
| 3D, batch (Gauss-Newton) | `python3 batch_3d.py` |
| 3D, incremental (ISAM2) | `python3 incremental_3d.py` |
