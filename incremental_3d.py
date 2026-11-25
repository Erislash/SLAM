import gtsam
import matplotlib.pyplot as plt
from reader import getVandE

vertexes, edges = getVandE("./data/parking-garage.g2o")

isam = gtsam.ISAM2()
result:gtsam.Values=gtsam.Values()
for vertex in sorted(vertexes):
    graph: gtsam.NonlinearFactorGraph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()

    if vertex.id == 0:

        prior_noise: gtsam.noiseModel.Diagonal = gtsam.noiseModel.Diagonal.Sigmas([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

        R_vertex: gtsam.Rot3 = gtsam.Rot3(vertex.qw, vertex.qx, vertex.qy, vertex.qz)
        t_vertex: gtsam.Point3 = gtsam.Point3(vertex.x, vertex.y, vertex.z)

        prior_pose: gtsam.Pose3 = gtsam.Pose3(R_vertex, t_vertex)

        prior_factor: gtsam.PriorFactorPose3 = gtsam.PriorFactorPose3(
            vertex.id, prior_pose, prior_noise
        )

        graph.add(prior_factor)

        init.insert(vertex.id, prior_pose)
    else:
        prevPose = result.atPose3(vertex.id - 1)
        init.insert(vertex.id, prevPose)

    for edge in edges:
        if edge.id_to == vertex.id:
            noise:gtsam.noiseModel.Gaussian = gtsam.noiseModel.Gaussian.Information(edge.weights)
            R_edge = gtsam.Rot3(edge.qw, edge.qx, edge.qy, edge.qz)
            t_edge = gtsam.Point3(edge.x, edge.y, edge.z)
            graph.add(gtsam.BetweenFactorPose3(edge.id_from, edge.id_to, gtsam.Pose3(R_edge, t_edge), noise))

    isam.update(graph, init)
    result = isam.calculateEstimate()

xs_init = [v.x for v in vertexes]
ys_init = [v.y for v in vertexes]
zs_init = [v.z for v in vertexes]

xs_opt = [pose.x() for pose in [result.atPose3(i) for i in range(result.size())]]
ys_opt = [pose.y() for pose in [result.atPose3(i) for i in range(result.size())]]
zs_opt = [pose.z() for pose in [result.atPose3(i) for i in range(result.size())]]

plt.plot(xs_init, ys_init, label="Trayectoria Inicial")
plt.plot(xs_opt, ys_opt, label="Trayectoria Optimizada")
plt.axis("equal")
plt.legend()
plt.title("Solución incremental 3D")
plt.xlabel("x")
plt.ylabel("y")
plt.savefig("figures/3d_incremental_xy.png", dpi=150)
plt.show()

plt.plot(xs_init, zs_init, label="Trayectoria Inicial")
plt.plot(xs_opt, zs_opt, label="Trayectoria Optimizada")
plt.legend()
plt.title("Solución incremental 3D")
plt.xlabel("x")
plt.ylabel("z")
plt.savefig("figures/3d_incremental_xz.png", dpi=150)
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
ax.set_title("Solución incremental 3D")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_zlim(-10, 10)
plt.savefig("figures/3d_incremental_xyz.png", dpi=150)
plt.show()