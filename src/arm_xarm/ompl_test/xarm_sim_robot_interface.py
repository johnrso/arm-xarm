import contextlib
import itertools
import os
import tempfile

import rospy
from loguru import logger
import numpy as np
import path
import pybullet as p
import pybullet_planning as pp

from . import geometry
from . import utils
from . import pybullet_utils
from .ompl_planning import PbPlanner
from scipy.spatial.transform import Rotation as R

import skrobot

from .utils import tsdf_from_depth

here = path.Path(__file__).abspath().parent


class XArmSimRobotInterface(object):

    def __init__(
        self,
        pose=None,
        planner="RRTConnect",
    ):
        self._3dmap = None

        pp.connect(use_gui=True)
        pp.add_data_path()
        pp.reset_simulation()
        pp.enable_gravity()

        self._collidables = []
        self._init_workspace()

        self.pose = pose
        tmp_file = here + '/urdf/xarm7_with_gripper.urdf'
        self.robot_model = skrobot.models.urdf.RobotModelFromURDF(
            urdf_file=tmp_file
        )
        self.robot_model.tipLink = self.robot_model.link_eef
        self.robot = pp.load_pybullet(
            tmp_file, fixed_base=True
        )

        self.ee = pp.link_from_name(self.robot, "link_eef")
        self.attachments = []
        if self.pose is not None:
            self.robot_model.translate(pose[0])
            self.robot_model.orient_with_matrix(
                geometry.quaternion_matrix(pose[1])[:3, :3]
            )
            pp.set_pose(self.robot, self.pose)

        # Get revolute joint indices of robot (skip fixed joints).
        n_joints = p.getNumJoints(self.robot)
        joints = [p.getJointInfo(self.robot, i) for i in range(n_joints)]
        joint_names = ["joint{}".format(i) for i in range(1, 8)]
        self.joints = [j[0] for j in joints if j[1].decode("utf-8") in joint_names]

        # self.homej = np.radians([
        #     180,
        #     0,
        #     -180,
        #     100,
        #     0,
        #     100,
        #     0,
        # ])
        self.homej = np.radians([
            180,
            50,
            -180,
            50,
            0,
            100,
            0,
        ])
        for joint, joint_angle in zip(self.joints, self.homej):
            p.resetJointState(self.robot, joint, joint_angle)
        self.update_robot_model()

        self.planner = planner

        lower, upper = self.get_bounds()
        for joint, min_angle, max_angle in zip(self.joints, lower, upper):
            joint_name = pp.get_joint_name(
                self.robot, joint
            ).decode()
            getattr(self.robot_model, joint_name).min_angle = min_angle
            getattr(self.robot_model, joint_name).max_angle = max_angle

    def update_env(self, depth, camera_to_base, K):
        tsdf = tsdf_from_depth(depth, camera_to_base, K)
        with tempfile.TemporaryDirectory() as tmp_dir:
            visual_file = path.Path(tmp_dir) / "tsdf.obj"
            tsdf.export(visual_file)
            collision_file = pybullet_utils.get_collision_file(
                visual_file, resolution=10000
            )
            _3dmap = pybullet_utils.create_mesh_body(
                visual_file=visual_file,
                collision_file=collision_file,
                rgba_color=(0.5, 0.5, 0.5, 1),
            )
            print('Added map to scene!')
            if self._3dmap is not None:
                p.removeBody(self._3dmap)
            self._3dmap = _3dmap

    def get_bounds(self):
        lower_bounds = []
        upper_bounds = []
        for joint in self.joints:
            lower, upper = p.getJointInfo(self.robot, joint)[8:10]
            center = (upper + lower) / 2
            width = upper - lower
            width = width * 0.96
            upper = center + width / 2
            lower = center - width / 2
            lower_bounds.append(lower)
            upper_bounds.append(upper)
        lower_bounds = np.array(lower_bounds)
        upper_bounds = np.array(upper_bounds)
        return lower_bounds, upper_bounds

    def step_simulation(self):
        self.gripper.step_simulation()

    def update_robot_model(self, j=None):
        if j is None:
            j = self.getj()
        for joint, joint_angle in zip(self.joints, j):
            joint_name = pp.get_joint_name(
                self.robot, joint
            ).decode()
            getattr(self.robot_model, joint_name).joint_angle(joint_angle)

    def setj(self, joint_positions):
        for joint, joint_position in zip(self.joints, joint_positions):
            p.resetJointState(self.robot, joint, joint_position)

    def getj(self):
        joint_positions = []
        for joint in self.joints:
            joint_positions.append(p.getJointState(self.robot, joint)[0])
        return joint_positions

    def movej(self, targj, speed=0.01, timeout=5, raise_on_timeout=False):
        assert len(targj) == len(self.joints)
        for i in itertools.count():
            currj = [p.getJointState(self.robot, i)[0] for i in self.joints]
            currj = np.array(currj)
            diffj = targj - currj
            if all(np.abs(diffj) < 1e-2):
                return

            # Move with constant velocity
            norm = np.linalg.norm(diffj)
            v = diffj / norm if norm > 0 else 0
            stepj = currj + v * speed
            gains = np.ones(len(self.joints))
            p.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=stepj,
                positionGains=gains,
            )
            yield i

            if i >= (timeout / pp.get_time_step()):
                if raise_on_timeout:
                    raise RuntimeError("timeout in joint motor control")
                else:
                    logger.error("timeout in joint motor control")
                    return

    def solve_ik(
        self,
        pose,
        move_target=None,
        n_init=1,
        random_state=None,
        validate=False,
        obstacles=None,
        **kwargs,
    ):

        # if obstacles is None:
        #     obstacles = []
        # obstacles.insert(0, self._3dmap)
        # obstacles += self._scene_objects
        obstacles = list(self._scene_objects)

        if move_target is None:
            move_target = self.robot_model.tipLink
        if random_state is None:
            random_state = np.random.RandomState()

        def sample_fn():
            lower, upper = self.get_bounds()
            extents = upper - lower
            scale = random_state.uniform(size=len(lower))
            return lower + scale * extents

        self.update_robot_model()
        c = geometry.Coordinate(*pose)
        for _ in range(n_init):
            result = self.robot_model.inverse_kinematics(
                c.skrobot_coords,
                move_target=move_target,
                **kwargs,
            )
            if result is not False:
                if not validate:
                    break
                if self.validatej(result, obstacles=obstacles):
                    break
            self.update_robot_model(sample_fn())
        else:
            # logger.warning("Failed to solve IK")
            return
        j = []
        for joint in self.joints:
            joint_name = pp.get_joint_name(
                self.robot, joint
            ).decode()
            j.append(getattr(self.robot_model, joint_name).joint_angle())
        return j

    # def _solve_ik_pybullet(self, pose):
    #     n_joints = p.getNumJoints(self.robot)
    #     lower_limits = []
    #     upper_limits = []
    #     for i in range(n_joints):
    #         joint_info = p.getJointInfo(self.robot, i)
    #         lower_limits.append(joint_info[8])
    #         upper_limits.append(joint_info[9])
    #     joint_positions = p.calculateInverseKinematics(
    #         bodyUniqueId=self.robot,
    #         endEffectorLinkIndex=self.ee,
    #         targetPosition=pose[0],
    #         targetOrientation=pose[1],
    #         lowerLimits=lower_limits,
    #         upperLimits=upper_limits,
    #         restPoses=self.homej,
    #         maxNumIterations=1000,
    #         residualThreshold=1e-5,
    #     )
    #     return joint_positions
    #
    # def _solve_ik_pybullet_planning(self, pose):
    #     with pp.LockRenderer():
    #         with pp.WorldSaver():
    #             joint_positions = pp.inverse_kinematics(
    #                 self.robot,
    #                 self.ee,
    #                 pose,
    #             )
    #     return joint_positions

    @contextlib.contextmanager
    def enabling_attachments(self):
        robot_model = self.robot_model
        try:
            self.robot_model = self.get_skrobot(attachments=self.attachments)
            yield
        finally:
            self.robot_model = robot_model

    def get_skrobot(self, attachments=None):
        attachments = attachments or []

        self.update_robot_model()

        link_list = self.robot_model.link_list.copy()
        joint_list = self.robot_model.joint_list.copy()
        for i, attachment in enumerate(attachments):
            position, quaternion = attachment.grasp_pose
            link = skrobot.model.Link(
                parent=self.robot_model.tipLink,
                pos=position,
                rot=geometry.quaternion_matrix(quaternion)[:3, :3],
                name=f"attachment_link{i}",
            )
            joint = skrobot.model.FixedJoint(
                child_link=link,
                parent_link=self.robot_model.tipLink,
                name=f"attachment_joint{i}",
            )
            link.joint = joint
            link_list.append(link)
            joint_list.append(joint)
        return skrobot.model.RobotModel(
            link_list=link_list,
            joint_list=joint_list,
            # root_link=self.robot_model.root_link,
        )

    def validatej(self, j, obstacles=None, min_distances=None):
        obstacles = list(self._scene_objects_ik)
        planner = PbPlanner(
            self,
            obstacles=obstacles,
            min_distances=min_distances,
            planner=self.planner,
        )
        print('Debug 3.8')
        return planner.validityChecker.isValid(j)

    def planj(
        self,
        j,
        obstacles=None,
        min_distances=None,
        min_distances_start_goal=None,
        planner_range=0,
    ):
        if self.planner == "Naive":
            return [j]

        # if obstacles is None:
        #     obstacles = []
        # obstacles.insert(0, self._3dmap)
        # obstacles += self._scene_objects
        obstacles = list(self._scene_objects)


        planner = PbPlanner(
            self,
            obstacles=obstacles,
            min_distances=min_distances,
            min_distances_start_goal=min_distances_start_goal,
            planner=self.planner,
            planner_range=planner_range,
        )

        planner.validityChecker.start = self.getj()
        planner.validityChecker.goal = j

        if not planner.validityChecker.isValid(self.getj()):
            # logger.warning("Start state is invalid")
            return

        if not planner.validityChecker.isValid(j):
            # logger.warning("Goal state is invalid")
            return

        result = planner.plan(self.getj(), j)

        if result is None:
            # logger.warning("No solution found")
            return

        ndof = len(self.joints)
        state_count = result.getStateCount()
        path = np.zeros((state_count, ndof), dtype=float)
        for i_state in range(state_count):
            state = result.getState(i_state)
            path_i = np.zeros((ndof,), dtype=float)
            for i_dof in range(ndof):
                path_i[i_dof] = state[i_dof]
            path[i_state] = path_i

        if not np.allclose(j, path[-1]):
            # the goal is not reached
            return

        return path

    # def grasp(self, min_dz=None, max_dz=None, rotation_axis="z", speed=0.01):
    #     c = geometry.Coordinate(
    #         *pp.get_link_pose(self.robot, self.ee)
    #     )
    #     dz_done = 0
    #     while True:
    #         c.translate([0, 0, 0.001])
    #         dz_done += 0.001
    #         j = self.solve_ik(c.pose, rotation_axis=rotation_axis)
    #         if j is None:
    #             raise RuntimeError("IK failed")
    #         for i in self.movej(j, speed=speed):
    #             yield i
    #             if min_dz is not None and dz_done < min_dz:
    #                 continue
    #             if self.gripper.detect_contact():
    #                 break
    #         if min_dz is not None and dz_done < min_dz:
    #             continue
    #         if self.gripper.detect_contact():
    #             break
    #         if max_dz is not None and dz_done >= max_dz:
    #             break
    #     self.gripper.activate()
    #
    # def ungrasp(self):
    #     self.gripper.release()
    #     # if hasattr(self, "virtual_grasped_object"):
    #     #     p.removeBody(self.virtual_grasped_object)
    #     #     del self.virtual_grasped_object
    #     self.attachments = []

    def add_link(self, name, pose, parent=None):
        if parent is None:
            parent = self.ee
        parent_name = pp.get_link_name(self.robot, parent)

        link_list = self.robot_model.link_list.copy()
        joint_list = self.robot_model.joint_list.copy()
        parent_link = getattr(self.robot_model, parent_name)
        link = skrobot.model.Link(
            parent=parent_link,
            pos=pose[0],
            rot=geometry.quaternion_matrix(pose[1])[:3, :3],
            name=name,
        )
        joint = skrobot.model.FixedJoint(
            child_link=link,
            parent_link=parent_link,
            name=f"{parent_name}_to_{name}_joint",
        )
        link.joint = joint
        link_list.append(link)
        joint_list.append(joint)
        self.robot_model = skrobot.model.RobotModel(
            link_list=link_list,
            joint_list=joint_list,
            # root_link=self.robot_model.root_link,
        )

    def get_pose(self, name):
        self.update_robot_model()
        T_a_to_world = getattr(self.robot_model, name).worldcoords().T()
        a_to_world = geometry.Coordinate.from_matrix(T_a_to_world).pose
        return a_to_world

    def add_camera(
        self,
        pose,
        fovy=np.deg2rad(42),
        height=480,
        width=640,
        parent=None,
    ):
        if parent is None:
            parent = self.ee
        self.add_link("camera_link", pose=pose, parent=parent)

        # pp.draw_pose(
        #     pose, parent=self.robot, parent_link=parent
        # )
        pybullet_utils.draw_camera(
            fovy=fovy,
            height=height,
            width=width,
            pose=pose,
            parent=self.robot,
            parent_link=parent,
        )

        self.camera = dict(fovy=fovy, height=height, width=width)

    def get_camera_image(self):
        if not hasattr(self.robot_model, "camera_link"):
            raise ValueError

        self.update_robot_model()
        return pybullet_utils.get_camera_image(
            T_cam2world=self.robot_model.camera_link.worldcoords().T(),
            fovy=self.camera["fovy"],
            height=self.camera["height"],
            width=self.camera["width"],
        )

    def get_opengl_intrinsic_matrix(self):
        return geometry.opengl_intrinsic_matrix(
            fovy=self.camera["fovy"],
            height=self.camera["height"],
            width=self.camera["width"],
        )

    # def random_grasp(
    #     self,
    #     depth,
    #     segm,
    #     mask,
    #     bg_object_ids,
    #     object_ids,
    #     max_angle=np.deg2rad(45),
    #     num_trial=10,
    #     random_state=None,
    #     noise=True,
    # ):
    #     if random_state is None:
    #         random_state = np.random.RandomState()
    #
    #     # This should be called after moving camera to observe the scene.
    #     K = self.get_opengl_intrinsic_matrix()
    #
    #     if mask.sum() == 0:
    #         logger.warning("mask is empty")
    #         return
    #
    #     pcd_in_camera = geometry.pointcloud_from_depth(
    #         depth, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
    #     )
    #
    #     camera_to_world = self.get_pose("camera_link")
    #     ee_to_world = self.get_pose("tipLink")
    #     camera_to_ee = pp.multiply(
    #         pp.invert(ee_to_world), camera_to_world
    #     )
    #     pcd_in_ee = geometry.transform_points(
    #         pcd_in_camera,
    #         geometry.transformation_matrix(*camera_to_ee),
    #     )
    #
    #     normals = geometry.normals_from_pointcloud(pcd_in_ee)
    #
    #     segm = segm.reshape(-1)
    #     mask = mask.reshape(-1)
    #     pcd_in_ee = pcd_in_ee.reshape(-1, 3)
    #     normals = normals.reshape(-1, 3)
    #
    #     indices = np.where(mask)[0]
    #     random_state.shuffle(indices)
    #
    #     j_init = self.getj()
    #
    #     path1 = path2 = None
    #     for index in indices[:num_trial]:
    #         object_id = segm[index]
    #         position = pcd_in_ee[index]
    #         quaternion = geometry.quaternion_from_vec2vec(
    #             [0, 0, 1], normals[index]
    #         )
    #         T_ee_to_ee_af_in_ee = geometry.transformation_matrix(
    #             position, quaternion
    #         )
    #
    #         T_ee_to_world = geometry.transformation_matrix(
    #             *pybullet_utils.get_pose(self.robot, self.ee)
    #         )
    #         T_ee_to_ee = np.eye(4)
    #         T_ee_af_to_ee = T_ee_to_ee_af_in_ee @ T_ee_to_ee
    #         T_ee_af_to_world = T_ee_to_world @ T_ee_af_to_ee
    #
    #         c = geometry.Coordinate(
    #             *geometry.pose_from_matrix(T_ee_af_to_world)
    #         )
    #         c.translate([0, 0, -0.1])
    #
    #         vec = geometry.transform_points([[0, 0, 0], [0, 0, 1]], c.matrix)
    #         if 0:
    #             pp.add_line(vec[0], vec[1], width=3)
    #         v0 = [0, 0, -1]
    #         v1 = vec[1] - vec[0]
    #         v1 /= np.linalg.norm(v1)
    #         angle = geometry.angle_between_vectors(v0, v1)
    #         if angle > max_angle:
    #             # logger.warning(f"angle > {np.rad2deg(max_angle)} deg")
    #             continue
    #
    #         j = self.solve_ik(
    #             c.pose, rotation_axis="z", random_state=random_state
    #         )
    #         if j is None:
    #             # logger.warning("j is None")
    #             continue
    #
    #         path1 = self.planj(j, obstacles=bg_object_ids + object_ids)
    #         if path1 is None:
    #             # logger.warning("path1 is None")
    #             continue
    #
    #         self.setj(j)
    #
    #         c.translate([0, 0, 0.1])
    #         j = self.solve_ik(
    #             c.pose, n_init=1, rotation_axis="z", random_state=random_state
    #         )
    #         if j is None:
    #             # logger.warning("j is None")
    #             self.setj(j_init)
    #             continue
    #
    #         obstacles = bg_object_ids + object_ids
    #         obstacles.remove(object_id)
    #         path2 = self.planj(j, obstacles=obstacles)
    #         if path2 is None:
    #             # logger.warning("path2 is None")
    #             self.setj(j_init)
    #             continue
    #
    #         self.setj(j_init)
    #         break
    #     if path1 is None or path2 is None:
    #         return
    #     for _ in (_ for j in path1 for _ in self.movej(j)):
    #         yield
    #
    #     # XXX: getting ground truth object pose
    #     obj_to_world = pp.get_pose(object_id)
    #     if noise:
    #         pos, qua = obj_to_world
    #         pos += random_state.normal(0, [0.003, 0.003, 0.003], 3)
    #         qua += random_state.normal(0, 0.01, 4)
    #         obj_to_world = pos, qua
    #
    #     for _ in self.grasp(min_dz=0.08, max_dz=0.12, speed=0.005):
    #         yield
    #
    #     obstacles = bg_object_ids + object_ids
    #     if self.gripper.check_grasp():
    #         obstacles.remove(object_id)
    #         ee_to_world = self.get_pose("tipLink")
    #         obj_to_ee = pp.multiply(
    #             pp.invert(ee_to_world), obj_to_world
    #         )
    #         self.attachments = [
    #             pp.Attachment(
    #                 self.robot, self.ee, obj_to_ee, object_id
    #             )
    #         ]
    #
    #
    #     else:
    #         self.attachments = []

    def move_to_homej(self, bg_object_ids, object_ids, speed=0.01, timeout=5):
        obstacles = bg_object_ids + object_ids
        if self.attachments and self.attachments[0].child in obstacles:
            obstacles.remove(self.attachments[0].child)

        js = None
        min_distance = 0
        while True:
            js = self.planj(
                self.homej,
                obstacles=obstacles,
                min_distances=utils.StaticDict(value=min_distance),
            )
            if js is not None:
                break

            if min_distance <= -0.05:
                js = [self.homej]
                break
            logger.warning(f"js is None w/ min_distance={min_distance}")
            min_distance -= 0.01
        for j in js:
            for _ in self.movej(j, speed=speed, timeout=timeout / len(js)):
                yield

    def get_cartesian_path(self, j=None, pose=None, rotation_axis=True):
        if not (j is None) ^ (pose is None):
            raise ValueError("Either j or coords must be given")

        p1 = self.get_pose("tipLink")

        with pp.WorldSaver():
            if j is None:
                j = self.solve_ik(pose, rotation_axis=rotation_axis)
                if j is None:
                    raise RuntimeError("IK failure")
            else:
                self.setj(j)
            j_final = j

            p2 = self.get_pose("tipLink")

        js = []
        for pose in pp.interpolate_poses(p1, p2):
            j = self.solve_ik(pose, rotation_axis=rotation_axis)
            if j is not None and self.validatej(j):
                js.append(j)
        if j_final is not None:
            js.append(j_final)
        js = np.array(js)
        return js
    
    def _init_workspace(self):
        self._scene_objects = []
        self._scene_objects_ik = []

        # light
        p.configureDebugVisualizer(
            p.COV_ENABLE_SHADOWS, True, lightPosition=(100, -100, 0.5)
        )

        table = pp.create_box(
            0.8, 1.4, 0.1, color=[150 / 255, 111 / 255, 51 / 255, 1]
        )
        # pp.set_pose(table, ([0.5, 0, 0.0], [0, 0, 0, 1]))
        pp.set_pose(table, ([0.5, 0, -0.07], [0, 0, 0, 1]))
        self._scene_objects.append(table)
        self._scene_objects_ik.append(table)


        # cabinet = pp.create_box(
        #     0.8, 0.4, 0.6, color=[50 / 255, 50 / 255, 50 / 255, 1]
        # )
        # pp.set_pose(cabinet, ([0.5, -0.6, 0.3], [0, 0, 0, 1]))
        # self._scene_objects.append(cabinet)
        # kitchen = pp.create_box(
        #     # 1.5, 0.4, 1.6, color=[50 / 255, 50 / 255, 50 / 255, 1]
        #     1.5, 0.45, 1.6, color=[50 / 255, 50 / 255, 50 / 255, 1]
        # )
        # pp.set_pose(kitchen, ([0.5, -0.6, 0.3], np.array(R.from_euler('z', 50, degrees=True).as_quat())))
        # self._scene_objects.append(kitchen)
        #
        # kitchen = pp.create_box(
        #     1.5, 0.3, 1.6, color=[50 / 255, 50 / 255, 50 / 255, 1]
        # )
        # pp.set_pose(kitchen, ([0.5, -0.6, 0.3], np.array(R.from_euler('z', 50, degrees=True).as_quat())))
        # self._scene_objects_ik.append(kitchen)

        # left wall
        obj = pp.create_box(w=3, l=0.01, h=1.05, color=(0.6, 0.6, 0.6, 1))
        pp.set_pose(
            obj,
            (
                (0.0, 0.35, 0.55),
                (0.0, 0.0, 0.0194987642109932, 0.9998098810245096),
            ),
        )
        self._scene_objects.append(obj)
        self._scene_objects_ik.append(obj)


        # back wall
        obj = pp.create_box(w=0.01, l=3, h=1.05, color=(0.7, 0.7, 0.7, 1))
        pp.set_pose(obj, ([0.8, 0, 1.05 / 2], [0, 0, 0, 1]))
        self._scene_objects.append(obj)
        self._scene_objects_ik.append(obj)

        # ceiling
        # obj = pp.create_box(w=3, l=3, h=0.5, color=(1, 1, 1, 1))
        # pp.set_pose(obj, ([0, 0, 0.25 + 1.05], [0, 0, 0, 1]))
        # self._scene_objects.append(obj)
