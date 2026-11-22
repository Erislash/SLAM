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



class Edge:
    def __init__(self, id_from:int, id_to:int, x:float, y:float, theta:float,
                 weights:NDArray[np.float64]) -> None:
        self.id_from = id_from
        self.id_to = id_to
        self.x = x
        self.y = y
        self.theta = theta
        self.weights = weights


def processVertex(id:str, x:str, y:str, theta:str) -> Vertex:
    vertex:Vertex = Vertex(int(id),
                    float(x),
                    float(y),
                    float(theta))
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


def getVandE(path:str) -> Tuple[List[Vertex], List[Edge]]:
    vertexes:List[Vertex] = []
    edges:List[Edge] = []

    with open(path, "r", encoding="utf-8") as file:
        for line_num, line in enumerate(file, start=1):

            if (len(line.strip().split()) == 0):
                continue

            tag:str = line.strip().split()[0]
            if tag == 'VERTEX_SE2':
                vertexes.append(processVertex(*line.strip().split()[1:]))
            elif tag == 'EDGE_SE2':
                edges.append(processEdge(*line.strip().split()[1:]))
            else:
                raise ValueError(
                  f"Tipo de registro inválido '{tag}' en la línea {line_num}"
                )
    return (vertexes, edges)