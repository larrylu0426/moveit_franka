from functools import wraps

from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import PlanningSceneComponents, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle, Plane, SolidPrimitive

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print(
            "Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info"
        )


class PlanningScene(Node):

    def __init__(self):
        super().__init__('planning_scene')
        self.cli_executor = SingleThreadedExecutor()
        self.cli_executor.add_node(self)

        self.apply_cli = self.create_client(ApplyPlanningScene,
                                            '/apply_planning_scene')
        self.get_cli = self.create_client(GetPlanningScene,
                                          '/get_planning_scene')

    def _process_future(self, future):
        self.cli_executor.spin_until_future_complete(future)
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().info(f"Service call failed {e}")
        return res

    def _make_mesh(self, name, pos, quat, filename, scale=(1, 1, 1)):
        co = CollisionObject()
        if pyassimp is False:
            raise Exception(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt"
            )
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise Exception("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise Exception("There are no faces in the mesh")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header.stamp = self.get_clock().now().to_msg()
        co.header.frame_id = "world"
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        co.pose = pose

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, "__len__"):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [
                        int(face[0]), int(face[1]),
                        int(face[2])
                    ]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, "indices"):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [
                        face.indices[0],
                        face.indices[1],
                        face.indices[2],
                    ]
                    mesh.triangles.append(triangle)
        else:
            raise Exception(
                "Unable to build triangles from mesh due to mesh object structure"
            )
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        pyassimp.release(scene)
        return co

    def _make_plane(self, name, pos, quat, normal, offset):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header.stamp = self.get_clock().now().to_msg()
        co.header.frame_id = "world"
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        co.pose = pose
        p = Plane()
        coef = list(normal) + [offset]
        p.coef = coef
        co.planes = [p]
        co.plane_poses = [pose]
        return co

    def _make_box(self, name, pos, quat, size):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header.stamp = self.get_clock().now().to_msg()
        co.header.frame_id = "world"
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        co.pose = pose
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        return co

    def check(func):

        @wraps(func)
        def wrapper(self, *args, **kwargs):
            clis = ['apply_cli', 'get_cli']
            for c in clis:
                s = getattr(self, c)
                if not s.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info("%s client not ready" % c)
                    return False
            return func(self, *args, **kwargs)

        return wrapper

    @check
    def get(self, scene_id=0):
        req = GetPlanningScene.Request()
        components = PlanningSceneComponents()
        components.components = scene_id
        req.components = components
        future = self.get_cli.call_async(req)
        res = self._process_future(future)
        return res.scene

    def attach(self, name, link):
        current_scene = self.get()
        cos = current_scene.world.collision_objects
        select_co = None
        for co in cos:
            if co.id == name:
                select_co = co
                break
        if not select_co:
            return False
        attached_co = AttachedCollisionObject()
        attached_co.link_name = link
        attached_co.object = co
        current_scene.robot_state.attached_collision_objects.append(
            attached_co)
        current_scene.is_diff = True
        current_scene.robot_state.is_diff = True
        req = ApplyPlanningScene.Request()
        req.scene = current_scene
        future = self.apply_cli.call_async(req)
        res = self._process_future(future)
        if not res.success:
            self.get_logger().error("Attach object %s failed." % name)
            return False
        else:
            self.get_logger().info("Attach object %s was successful" % name)
            return True

    @check
    def add_mesh(self, name, pos, quat, filename):
        current_scene = self.get()
        new_object = self._make_mesh(name, pos, quat, filename)
        current_scene.world.collision_objects.append(new_object)
        req = ApplyPlanningScene.Request()
        req.scene = current_scene
        future = self.apply_cli.call_async(req)
        res = self._process_future(future)
        if not res.success:
            self.get_logger().error("Add mesh object failed.")
            return False
        else:
            self.get_logger().info("Add mesh object was successful")
            return True

    @check
    def add_plane(self, name, pos, quat, normal=(0.0, 0.0, 1.0), offset=0.0):
        current_scene = self.get()
        new_object = self._make_plane(name, pos, quat, normal, offset)
        current_scene.world.collision_objects.append(new_object)
        req = ApplyPlanningScene.Request()
        req.scene = current_scene
        future = self.apply_cli.call_async(req)
        res = self._process_future(future)
        if not res.success:
            self.get_logger().error("Add plane failed.")
            return False
        else:
            self.get_logger().info("Add plane was successful")
            return True

    @check
    def add_box(self, name, pos, quat, size=(1.0, 1.0, 1.0)):
        current_scene = self.get()
        new_object = self._make_box(name, pos, quat, size)
        current_scene.world.collision_objects.append(new_object)
        req = ApplyPlanningScene.Request()
        req.scene = current_scene
        future = self.apply_cli.call_async(req)
        res = self._process_future(future)
        if not res.success:
            self.get_logger().error("Add box failed.")
            return False
        else:
            self.get_logger().info("Add box was successful")
            return True
