import pybullet as p
import time
import numpy as np




def initializeEnv(pb_client):

    pb_client.setGravity(0, 0, -10)

    cube1 = Cuboid(pb_client, length=100, width=100, height=1, x=0, y=0, z=-1)
    cube1.create()

    # pb_client.loadURDF("plane.urdf")
    # pb_client.loadURDF("table/table.urdf", [0,0,-1], globalScaling=3)




class IterRegistry(type):
    def __len__(cls):
        return len(cls._registry)

    def __iter__(cls):
        return iter(cls._registry)

    def __reversed__(cls):
        return reversed(cls._registry)

class Cuboid:
    _registry = []

    def __init__(self, physics_client, length=1.0, width=1.0, height=1.0, x=0, y=0, z=0):
        self._registry.append(self)
        self.startPos = None
        self.startOrn = None
        self.length = length
        self.width = width
        self.height = height
        self.size = [length, width, height]
        self.pos = [x, y, z]
        self.body_id = None
        self.pb_client = physics_client
        self.startOrn = None

    def create(self):
        half_extents = [self.length / 2, self.width / 2, self.height / 2]
        self.body_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        p.createMultiBody(baseMass=1, baseCollisionShapeIndex=self.body_id, basePosition=self.pos)

    def remove(self):
        p.removeBody(self.body_id)

    def get_pose(self):
        return self.pb_client.getBasePositionAndOrientation(self.body_id)

    def set_assembly_pose(self):
        if self.body_id == None:
            print("cube not loaded ...")
            return
        # self.pb_client.resetBasePositionAndOrientation(
        #     self.body_id,
        #     self.pos,
        #     self.size)

    def reset_start_pose(self):
        self.pb_client.resetBasePositionAndOrientation(
            self.body_id,
            self.startPos,
            self.startOrn)
    #
    def move_cube(self):
        alpha = 4
        force = alpha*np.array([0, 0, 3])
        current_pos, _ = self.get_pose()
        self.pb_client.applyExternalForce(
            objectUniqueId=self.body_id, linkIndex=-1,
            forceObj=force, posObj=current_pos,
            flags=p.WORLD_FRAME)


def initialCube(pb_client):
    Cuboid._registry = []
    for i in range(10):
        cuboid = Cuboid(pb_client, length=8, width=8, height=1.0, x=0, y=0, z=i * 2)
        cuboid.create()
        time.sleep(1)


    #
    # length_t=8
    # width_t=8
    # height_t=8
    #
    # cubeT = Cuboid(pb_client,40, 40, 10, x=0, y=10, z=0)
    # cubeL = Cuboid(pb_client,length_t, width_t, height_t, x=0, y=10, z=20)
    # cubeZ = Cuboid(pb_client,length_t, width_t, height_t, x=0, y=30, z=0)
    # cubeV = Cuboid(pb_client,length_t, width_t, height_t, x=0, y=40, z=0)
    # cubeB = Cuboid(pb_client,length_t, width_t, height_t, x=0, y=50, z=0)
    # cubeA = Cuboid(pb_client,length_t, width_t, height_t, x=0, y=60, z=0)
    # cubeP = Cuboid(pb_client,length_t, width_t, height_t, x=0, y=70, z=0)
    #
    # cubeT.create()
    # cubeL.create()
    # cubeZ.create()
    # cubeV.create()
    # cubeB.create()
    # cubeA.create()
    # cubeP.create()

    return Cuboid


def collision_analysis(direction, pb_client):
    """Detects pairwise collision during disassembly"""
    initializeEnv(pb_client)
    cube_class = initialCube(pb_client)
    collision_map = np.zeros((len(cube_class), len(cube_class)))

    for cube1 in cube_class:
        for cube2 in cube_class:
            # place two cubes in their assembly pose
            if cube1.boxId == cube2.boxId:
                continue
            cube1.set_assembly_pose()
            cube2.set_assembly_pose()
            # run step simulation with one cube fixed
            for _ in range(100):
                cube1.set_assembly_pose()
                cube2.move_cube()
                time.sleep(1./120.)
                pb_client.stepSimulation()
            # check if there's a position change for the moving cube
            finalPos, _ = cube2.get_pose()
            diff = np.array(finalPos) - np.array(cube2.pos)
            if (diff[0]**2 + diff[1]**2) > 0.1 or diff[2] < 1:
                collision_map[cube2.boxId - 2, cube1.boxId - 2] = 1
                print(f"cube[boxId:{cube2.boxId}] collides with",
                      f"cube[boxId:{cube1.boxId}]")
            # reset the moving cube to its start pose
            cube2.reset_start_pose()
        cube1.reset_start_pose()

    pb_client.resetSimulation()
    return collision_map





    for L in tqdm(range(len(cube_class))):
        print("1")
        for subset in combinations(cube_class._registry, L + 1):
            #subsetId = []
            is_stable = True
            # place a combination of cubes in their assembly pose
            for cube in subset:
                cube.set_assembly_pose()
                #subsetId.append(cube.boxId - 2)
            # run step simulation
            for _ in range(200):
                pb_client.stepSimulation()
                time.sleep(1. / 240.)

            # check if there's a position change for each cube in the subset
            for cube in subset:
                finalPos, _ = cube.get_pose()
                diff = np.array(finalPos) - np.array(cube.pos)
                if norm(diff) > 0.03:
                    is_stable = False
            # reset cubes to their start pose
            # for cube in subset:
            #     cube.reset_start_pose()
            # if is_stable:
            #     stable_subsets.append(subsetId)





def collision_check(p, s, cm):
    """Checks collision between part p and subassembly s"""
    num_collision = 0

    for i in range(6): # check if p collides with any part in the subassembly
        has_collision = np.any(cm[p, s, i])
        if has_collision: num_collision += 1

    return num_collision


def distance(indice1, indice2, s, cm):
    """Computes pairwise distance matrix between subassemblies"""
    dist = 10
    s1 = set(s[indice1])
    s2 = set(s[indice2])

    if s1.issubset(s2): # combinations() returns results in sorted order
        if len(s2) - len(s1) == 1:
            p = s2 - s1
            p = list(p)[0] # retrieve the element
            s1 = list(s1)
            num_collisions = collision_check(p, s1, cm)
            dist = num_collisions

    return dist

