import gtsam
import matplotlib.pyplot as plt
from reader import getVandE

vertexes, edges = getVandE("./data/input_INTEL_g2o.g2o")

graph: gtsam.NonlinearFactorGraph = gtsam.NonlinearFactorGraph()

pose0 = vertexes[0]

prior_noise: gtsam.noiseModel.Diagonal = gtsam.noiseModel.Diagonal.Sigmas([0.001, 0.001, 0.001])

prior_pose: gtsam.Pose2 = gtsam.Pose2(pose0.x, pose0.y, pose0.theta)

prior_factor: gtsam.PriorFactorPose2 = gtsam.PriorFactorPose2(
    pose0.id, prior_pose, prior_noise
)

graph.add(prior_factor)


for edge in edges:
    noise:gtsam.noiseModel.Gaussian = gtsam.noiseModel.Gaussian.Information(edge.weights)
    graph.add(gtsam.BetweenFactorPose2(edge.id_from, edge.id_to, gtsam.Pose2(edge.x, edge.y, edge.theta), noise))


# print(graph.size())


init = gtsam.Values()

for vertex in vertexes:
    init.insert(vertex.id, gtsam.Pose2(vertex.x, vertex.y, vertex.theta))

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

xs_opt = [pose.x() for pose in [optimum.atPose2(i) for i in range(init.size())]]
ys_opt = [pose.y() for pose in [optimum.atPose2(i) for i in range(init.size())]]

plt.plot(xs_init, ys_init, label="Trayectoria Inicial")
plt.plot(xs_opt, ys_opt, label="Trayectoria Optimizada")
plt.axis("equal")
plt.legend()
plt.title("Solución batch 2D")
plt.xlabel("x")
plt.ylabel("y")
plt.savefig("figures/2d_batch.png", dpi=150)
plt.show()