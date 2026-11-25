import gtsam
import matplotlib.pyplot as plt
from reader import getVandE

vertexes, edges = getVandE("./data/parking-garage.g2o")

graph: gtsam.NonlinearFactorGraph = gtsam.NonlinearFactorGraph()

pose0 = vertexes[0]
R0 = gtsam.Rot3(pose0.qw, pose0.qx, pose0.qy, pose0.qz)
t0 = gtsam.Point3(pose0.x, pose0.y, pose0.z)

prior_noise: gtsam.noiseModel.Diagonal = gtsam.noiseModel.Diagonal.Sigmas([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

prior_pose: gtsam.Pose3 = gtsam.Pose3(R0, t0)

prior_factor: gtsam.PriorFactorPose3 = gtsam.PriorFactorPose3(
    pose0.id, prior_pose, prior_noise
)

graph.add(prior_factor)


for edge in edges:
    noise:gtsam.noiseModel.Gaussian = gtsam.noiseModel.Gaussian.Information(edge.weights)
    R_edge = gtsam.Rot3(edge.qw, edge.qx, edge.qy, edge.qz)
    t_edge = gtsam.Point3(edge.x, edge.y, edge.z)
    graph.add(gtsam.BetweenFactorPose3(edge.id_from, edge.id_to, gtsam.Pose3(R_edge, t_edge), noise))

print(graph.size())


init = gtsam.Values()

for vertex in vertexes:
    R_vertex = gtsam.Rot3(vertex.qw, vertex.qx, vertex.qy, vertex.qz)
    t_vertex = gtsam.Point3(vertex.x, vertex.y, vertex.z)
    init.insert(vertex.id, gtsam.Pose3(R_vertex, t_vertex))

# print(init.size())
# print(graph.error(init))
params = gtsam.GaussNewtonParams()
# params.setVerbosity("ERROR")
optimizer = gtsam.GaussNewtonOptimizer(graph, init, params)
optimum:gtsam.Values = optimizer.optimize()


print(graph.error(optimum))
print(optimizer.iterations())


xs_init = [v.x for v in vertexes]
ys_init = [v.y for v in vertexes]
zs_init = [v.z for v in vertexes]

xs_opt = [pose.x() for pose in [optimum.atPose3(i) for i in range(init.size())]]
ys_opt = [pose.y() for pose in [optimum.atPose3(i) for i in range(init.size())]]
zs_opt = [pose.z() for pose in [optimum.atPose3(i) for i in range(init.size())]]

plt.plot(xs_init, ys_init, label="Trayectoria Inicial")
plt.plot(xs_opt, ys_opt, label="Trayectoria Optimizada")
plt.axis("equal")
plt.legend()
plt.title("Solución batch 3D")
plt.xlabel("x")
plt.ylabel("y")
plt.savefig("figures/3d_batch_xy.png", dpi=150)
plt.show()

plt.plot(xs_init, zs_init, label="Trayectoria Inicial")
plt.plot(xs_opt, zs_opt, label="Trayectoria Optimizada")
plt.legend()
plt.title("Solución batch 3D")
plt.xlabel("x")
plt.ylabel("z")
plt.savefig("figures/3d_batch_xz.png", dpi=150)
plt.show()

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot(xs_init, ys_init, zs_opt, label="Trayectoria Inicial")
ax.plot(xs_opt, ys_opt, zs_opt, label="Trayectoria Optimizada")
ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.view_init(30, -120)
ax.legend()
ax.set_title("Solución batch 3D")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_zlim(-10, 10)
plt.savefig("figures/3d_batch_xyz.png", dpi=150)
plt.show()