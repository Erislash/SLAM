import gtsam
import matplotlib.pyplot as plt
from reader import getVandE

vertexes, edges = getVandE("./data/input_INTEL_g2o.g2o")

isam = gtsam.ISAM2()
result:gtsam.Values=gtsam.Values()
for vertex in sorted(vertexes):
    graph: gtsam.NonlinearFactorGraph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()

    if vertex.id == 0:

        prior_noise: gtsam.noiseModel.Diagonal = gtsam.noiseModel.Diagonal.Sigmas([0.001, 0.001, 0.001])

        prior_pose: gtsam.Pose2 = gtsam.Pose2(vertex.x, vertex.y, vertex.theta)

        prior_factor: gtsam.PriorFactorPose2 = gtsam.PriorFactorPose2(
            vertex.id, prior_pose, prior_noise
        )

        graph.add(prior_factor)

        init.insert(vertex.id, gtsam.Pose2(vertex.x, vertex.y, vertex.theta))
    else:
        prevPose = result.atPose2(vertex.id - 1)
        init.insert(vertex.id, prevPose)

    for edge in edges:
        if edge.id_to == vertex.id:
            noise:gtsam.noiseModel.Gaussian = gtsam.noiseModel.Gaussian.Information(edge.weights)
            graph.add(gtsam.BetweenFactorPose2(edge.id_from, edge.id_to, gtsam.Pose2(edge.x, edge.y, edge.theta), noise))

    isam.update(graph, init)
    result = isam.calculateEstimate()

xs_init = [v.x for v in vertexes]
ys_init = [v.y for v in vertexes]

xs_opt = [pose.x() for pose in [result.atPose2(i) for i in range(result.size())]]
ys_opt = [pose.y() for pose in [result.atPose2(i) for i in range(result.size())]]

plt.plot(xs_init, ys_init, label="Trayectoria Inicial")
plt.plot(xs_opt, ys_opt, label="Trayectoria Optimizada")
plt.axis("equal")
plt.legend()
plt.title("Solución incremental 2D")
plt.xlabel("x")
plt.ylabel("y")
plt.savefig("figures/2d_incremental.png", dpi=150)
plt.show()
