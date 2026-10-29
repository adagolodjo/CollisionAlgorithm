"""A flexible rod (a chain of mass points joined by springs) falls onto a fixed triangle surface and rests on it.

Shows the contact pipeline of the two plugins with a thin object that is not a needle:
  CollisionLoop          runs the detection at the beginning of every step
  EdgeGeometry           the segments of the rod            TriangleGeometry   the surface (+ AABBBroadPhase)
  ClosestProximityAlgorithm   for each segment, the closest point of the surface (and, projectively, the point of the
                         segment closest to the surface), kept when closer than `distance`
  SecondDirection + ConstraintUnilateral(contactDistance = radius of the rod, mu = friction)
Units: mm, kg, s.   runSofa -l CollisionAlgorithm -l ConstraintGeometry RodOnSurface.py
"""
import Sofa

ROD_RADIUS = 1.0
POINTS = 11
SEGMENT = 10.0


def createScene(root):
    root.gravity = [0, 0, -9810]
    root.dt = 0.005
    root.addObject("RequiredPlugin", pluginName=["Sofa.Component.AnimationLoop", "Sofa.Component.Constraint.Lagrangian.Solver",
                                                 "Sofa.Component.Constraint.Lagrangian.Correction",
                                                 "Sofa.Component.IntegrationScheme.Backward", "Sofa.Component.LinearSolver.Direct",
                                                 "Sofa.Component.Mass", "Sofa.Component.SolidMechanics.Spring",
                                                 "Sofa.Component.StateContainer", "Sofa.Component.Topology.Container.Dynamic",
                                                 "Sofa.Component.Visual", "Sofa.GL.Component.Rendering3D",
                                                 "CollisionAlgorithm", "ConstraintGeometry"])
    root.addObject("VisualStyle", displayFlags="showVisualModels showBehaviorModels showCollisionModels showInteractionForceFields")
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("BlockGaussSeidelConstraintSolver", tolerance=1e-8, maxIterations=200)
    root.addObject("CollisionLoop")

    positions = [[20.0 + i * SEGMENT * 0.9, 0.0, 6.0 + 0.5 * i] for i in range(POINTS)]   # tilted: one end touches first
    rod = root.addChild("Rod")
    rod.addObject("EulerImplicitIntegrationScheme", rayleighStiffness=0.1)
    rod.addObject("SparseLDLSolver", name="linearSolver", template="CompressedRowSparseMatrixd")
    rod.addObject("EdgeSetTopologyContainer", name="topology", position=positions,
                  edges=[[i, i + 1] for i in range(POINTS - 1)])
    rod.addObject("MechanicalObject", name="state", template="Vec3d", position=positions)
    rod.addObject("UniformMass", totalMass=0.01)
    rod.addObject("MeshSpringForceField", stiffness=2e4, damping=1)
    rod.addObject("GenericConstraintCorrection", linearSolver="@linearSolver")
    rod.addObject("EdgeGeometry", name="geometry", mstate="@state", topology="@topology")

    surface = root.addChild("Surface")
    corners = [[-50, -50, 0], [150, -50, 0], [150, 50, 0], [-50, 50, 0]]
    surface.addObject("TriangleSetTopologyContainer", name="topology", position=corners, triangles=[[0, 1, 2], [0, 2, 3]])
    surface.addObject("MechanicalObject", name="state", template="Vec3d", position=corners)
    surface.addObject("TriangleGeometry", name="geometry", mstate="@state", topology="@topology")
    surface.addObject("PhongTriangleNormalHandler", name="normals", geometry="@geometry")
    surface.addObject("AABBBroadPhase", name="broadphase", geometry="@geometry", isStatic=True, method=2, nbox=[4, 4, 1])

    root.addObject("ClosestProximityAlgorithm", name="detection", dest="@Surface/geometry", distance=ROD_RADIUS + 2.0,
                   **{"from": "@Rod/geometry"})
    root.addObject("SecondDirection", name="direction", handler="@Surface/normals")
    root.addObject("ConstraintUnilateral", name="contact", input="@detection.collisionOutput", directions="@direction",
                   mu=0.3, frictionCone=True, contactDistance=ROD_RADIUS)
    return root
