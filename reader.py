from typing import Tuple, List

import numpy as np
from numpy.typing import NDArray


class Vertex:
    def __init__(self, id:int, x:float, y:float, theta:float) -> None:
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self) -> str:
        return f'Vertex(id: {self.id}, x: {self.x}, y: {self.y}, θ: {self.theta})'

    def __lt__(self, other:object) -> bool:
        if not isinstance(other, Vertex):
            return NotImplemented

        return self.id < other.id

class Vertex3D:
    def __init__(self, id:int, x:float, y:float, z:float,
                 qx:float, qy:float, qz:float, qw:float) -> None:
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw

    def __str__(self) -> str:
        return f'Vertex(id: {self.id}, x: {self.x}, y: {self.y}, z: {self.z}, qx: {self.qx}, qy: {self.qy}, qz: {self.qz}, qw: {self.qw})'

    def __lt__(self, other:object) -> bool:
        if not isinstance(other, Vertex3D):
            return NotImplemented

        return self.id < other.id

class Edge:
    def __init__(self, id_from:int, id_to:int, x:float, y:float, theta:float,
                 weights:NDArray[np.float64]) -> None:
        self.id_from = id_from
        self.id_to = id_to
        self.x = x
        self.y = y
        self.theta = theta
        self.weights = weights

class Edge3D:
    def __init__(self, id_from:int, id_to:int, x:float, y:float, z:float,
                 qx:float, qy:float, qz:float, qw:float,
                 weights:NDArray[np.float64]) -> None:
        self.id_from = id_from
        self.id_to = id_to
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.weights = weights

def processVertex(id:str, x:str, y:str, theta:str) -> Vertex:
    vertex:Vertex = Vertex(int(id),
                    float(x),
                    float(y),
                    float(theta))
    return vertex
    
def processVertex3D(id:str, x:str, y:str, z:str, qx:str, qy:str, qz:str, qw:str) -> Vertex3D:
    vertex:Vertex3D = Vertex3D(int(id),
                        float(x),
                        float(y),
                        float(z),
                        float(qx),
                        float(qy),
                        float(qz),
                        float(qw))
    return vertex

def processEdge(id_from:str, id_to:str, dx:str, dy:str, dtheta:str,
                i11:str, i12:str, i13:str, i22:str, i23:str, i33:str) -> Edge:
    weights = np.array([
        [float(i11), float(i12), float(i13)],
        [float(i12), float(i22), float(i23)],
        [float(i13), float(i23), float(i33)],
    ])

    edge = Edge(int(id_from),
                int(id_to),
                float(dx),
                float(dy),
                float(dtheta),
                weights)
    return edge

def processEdge3D(id_from:str, id_to:str, dx:str, dy:str, dz:str,
                  dqx:str, dqy:str, dqz:str, dqw:str,
                  i11:str, i12:str, i13:str, i14:str, i15:str, i16:str,
                  i22:str, i23:str, i24:str, i25:str, i26:str, i33:str,
                  i34:str, i35:str, i36:str, i44:str, i45:str, i46:str,
                  i55:str, i56:str, i66:str) -> Edge3D:
    weights = np.array([
        [float(i11), float(i12), float(i13), float(i14), float(i15), float(i16)],
        [float(i12), float(i22), float(i23), float(i24), float(i25), float(i26)],
        [float(i13), float(i23), float(i33), float(i34), float(i35), float(i36)],
        [float(i14), float(i24), float(i34), float(i44), float(i45), float(i46)],
        [float(i15), float(i25), float(i35), float(i45), float(i55), float(i56)],
        [float(i16), float(i26), float(i36), float(i46), float(i56), float(i66)]
    ])

    edge = Edge3D(int(id_from),
                int(id_to),
                float(dx),
                float(dy),
                float(dz),
                float(dqx),
                float(dqy),
                float(dqz),
                float(dqw),
                weights)
    return edge


def getVandE(path:str) -> Tuple[List[Vertex | Vertex3D], List[Edge | Edge3D]]:
    vertexes:List[Vertex | Vertex3D] = []
    edges:List[Edge | Edge3D] = []

    with open(path, "r", encoding="utf-8") as file:
        for line_num, line in enumerate(file, start=1):

            if (len(line.strip().split()) == 0):
                continue

            tag:str = line.strip().split()[0]
            if tag == 'VERTEX_SE2':
                vertexes.append(processVertex(*line.strip().split()[1:]))
            elif tag == 'EDGE_SE2':
                edges.append(processEdge(*line.strip().split()[1:]))
            elif tag == 'VERTEX_SE3:QUAT':
                vertexes.append(processVertex3D(*line.strip().split()[1:]))
            elif tag == 'EDGE_SE3:QUAT':
                edges.append(processEdge3D(*line.strip().split()[1:]))
            else:
                raise ValueError(
                  f"Tipo de registro inválido '{tag}' en la línea {line_num}"
                )
    return (vertexes, edges)