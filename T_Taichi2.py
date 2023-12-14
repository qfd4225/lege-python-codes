import taichi as ti
from taichi.math import *
import math
from scene import Scene

ti.init(arch=ti.gpu)  # Alternatively, ti.init(arch=ti.cpu)



num_particles = 2000
dim = 3
world_scale_factor = 1.0 / 100.0
dt = 1e-2
mass_inv = 1.0

positions = ti.Vector.field(dim, float, num_particles)
velocities = ti.Vector.field(dim, float, num_particles)
pos_draw = ti.Vector.field(dim, float, num_particles)
force = ti.Vector.field(dim, float, num_particles)
positions0 = ti.Vector.field(dim, float, num_particles)
radius_vector = ti.Vector.field(dim, float, num_particles)
paused = ti.field(ti.i32, shape=())
q_inv = ti.Matrix.field(n=3, m=3, dtype=float, shape=())


@ti.kernel
def init_particles():
    init_pos = ti.Vector([70.0, 50.0, 0.0])
    cube_size = 20
    spacing = 2
    num_per_row = (int)(cube_size // spacing) + 1
    num_per_floor = num_per_row * num_per_row
    for i in range(num_particles):
        floor = i // (num_per_floor)
        row = (i % num_per_floor) // num_per_row
        col = (i % num_per_floor) % num_per_row
        positions[i] = ti.Vector([col * spacing, floor * spacing, row * spacing]) + init_pos


@ti.kernel
def shape_matching():
    #  update vel and pos firtly
    gravity = ti.Vector([0.0, -9.8, 0.0])
    for i in range(num_particles):
        positions0[i] = positions[i]
        f = gravity
        velocities[i] += mass_inv * f * dt
        positions[i] += velocities[i] * dt
        if positions[i].y < 0.0:
            positions[i] = positions0[i]
            positions[i].y = 0.0

    # compute the new(matched shape) mass center
    c = ti.Vector([0.0, 0.0, 0.0])
    for i in range(num_particles):
        c += positions[i]
    c /= num_particles

    # compute transformation matrix and extract rotation
    A = sum1 = ti.Matrix([[0.0] * 3 for _ in range(3)], ti.f32)
    for i in range(num_particles):
        sum1 += (positions[i] - c).outer_product(radius_vector[i])
    A = sum1 @ q_inv[None]

    R, _ = ti.polar_decompose(A)

    # update velocities and positions
    for i in range(num_particles):
        positions[i] = c + R @ radius_vector[i]
        velocities[i] = (positions[i] - positions0[i]) / dt


@ti.kernel
def compute_radius_vector():
    # compute the mass center and radius vector
    center_mass = ti.Vector([0.0, 0.0, 0.0])
    for i in range(num_particles):
        center_mass += positions[i]
    center_mass /= num_particles
    for i in range(num_particles):
        radius_vector[i] = positions[i] - center_mass


@ti.kernel
def precompute_q_inv():
    res = ti.Matrix([[0.0] * 3 for _ in range(3)], ti.f64)
    for i in range(num_particles):
        res += radius_vector[i].outer_product(radius_vector[i])
    q_inv[None] = res.inverse()


@ti.kernel
def rotation(angle: ti.f32):
    theta = angle / 180.0 * math.pi
    R = ti.Matrix([
        [ti.cos(theta), -ti.sin(theta), 0.0],
        [ti.sin(theta), ti.cos(theta), 0.0],
        [0.0, 0.0, 1.0]
    ])
    for i in range(num_particles):
        positions[i] = R @ positions[i]


# ---------------------------------------------------------------------------- #
#                                    substep                                   #
# ---------------------------------------------------------------------------- #
def substep1():
    shape_matching()

def substep2():
    for i in ti.grouped(x):
        v[i] += gravity * dt

    for i in ti.grouped(x):
        force = ti.Vector([0.0, 0.0, 0.0])
        for spring_offset in ti.static(spring_offsets):
            j = i + spring_offset
            if 0 <= j[0] < n and 0 <= j[1] < n:
                x_ij = x[i] - x[j]
                v_ij = v[i] - v[j]
                d = x_ij.normalized()
                current_dist = x_ij.norm()
                original_dist = quad_size * float(i - j).norm()
                # Spring force
                force += -spring_Y * d * (current_dist / original_dist - 1)
                # Dashpot damping
                force += -v_ij.dot(d) * d * dashpot_damping * quad_size

        v[i] += force * dt
    #
    # for i in ti.grouped(x):
    #     v[i] *= ti.exp(-drag_damping * dt)
    #     offset_to_center = x[i] - ball_center[0]
    #     if offset_to_center.norm() <= ball_radius:
    #         # Velocity projection
    #         normal = offset_to_center.normalized()
    #         v[i] -= min(v[i].dot(normal), 0) * normal
    #     x[i] += dt * v[i]


# ---------------------------------------------------------------------------- #
#                                  end substep                                 #
# ---------------------------------------------------------------------------- #

@ti.kernel
def world_scale():
    for i in range(num_particles):
        pos_draw[i] = positions[i] * world_scale_factor

# # init the window, canvas, scene and camerea
# window = ti.ui.Window("rigidbody", (1280, 720), vsync=True)
# canvas = window.get_canvas()
# scene = ti.ui.Scene()
# camera = ti.ui.Camera()
#
# # initial camera position
# camera.position(0.5, 1.0, 1.95)
# camera.lookat(0.5, 0.3, 0.5)
# camera.fov(55)






# scene = Scene(voxel_edges=0, exposure=2)
# scene.set_floor(-0.85, (1.0, 1.0, 1.0))
# scene.set_background_color((0.5, 0.5, 0.4))
# scene.set_directional_light((1, 1, -1), 0.2, (1, 0.8, 0.6))
#

@ti.func
def create_block(pos, size, color, color_noise):
    for I in ti.grouped(
            ti.ndrange((pos[0], pos[0] + size[0]), (pos[1], pos[1] + size[1]),
                       (pos[2], pos[2] + size[2]))):
            scene.set_voxel(I, 1, color )


@ti.kernel
def initialize_voxels():
    create_block(ivec3(-60, -50, -60), ivec3(120, 100, 120), vec3(0.3, 0.2, 0.1),
                 vec3(0.01))
#initialize_voxels()
#scene.finish()


n = 128
quad_size = 1.0 / n
dt = 4e-2 / n
substeps = int(1 / 60 // dt)

gravity = ti.Vector([0, -9.8, 0])
spring_Y = 3e4
dashpot_damping = 1e4
drag_damping = 1

# Use a 1D field for storing the position of the ball center
# The only element in the field is a 3-dimentional floating-point vector
# Place the ball center at the original point

#球体的数据结构
ball_radius = 0.5
ball_center = ti.Vector.field(3, dtype=float, shape=(1, ))
ball_center[0] = [0, 0, 0]

#声明两个数组 x 和 v，用于存储质点的位置和速度。 Taichi 将这样的数组命名为 field。
x = ti.Vector.field(3, dtype=float, shape=(n, n))
v = ti.Vector.field(3, dtype=float, shape=(n, n))

num_triangles = (n - 1) * (n - 1) * 2
indices = ti.field(int, shape=num_triangles * 3)
vertices = ti.Vector.field(3, dtype=float, shape=n * n)
colors = ti.Vector.field(3, dtype=float, shape=n * n)

bending_springs = False


#重要——————————————————————————————————————————————————————————————————————————————————————————

@ti.kernel
def initialize_mass_points():
    random_offset = ti.Vector([ti.random() - 0.5, ti.random() - 0.5]) * 0.1

    for i, j in x:
        x[i, j] = [
            i * quad_size - 0.5 + random_offset[0], 0.6,
            j * quad_size - 0.5 + random_offset[1]
        ]
        v[i, j] = [0, 0, 0]



@ti.kernel
def initialize_mesh_indices():
    for i, j in ti.ndrange(n - 1, n - 1):
        quad_id = (i * (n - 1)) + j
        # 1st triangle of the square
        indices[quad_id * 6 + 0] = i * n + j
        indices[quad_id * 6 + 1] = (i + 1) * n + j
        indices[quad_id * 6 + 2] = i * n + (j + 1)
        # 2nd triangle of the square
        indices[quad_id * 6 + 3] = (i + 1) * n + j + 1
        indices[quad_id * 6 + 4] = i * n + (j + 1)
        indices[quad_id * 6 + 5] = (i + 1) * n + j

    for i, j in ti.ndrange(n, n):
        if (i // 4 + j // 4) % 2 == 0:
            colors[i * n + j] = (0.22, 0.72, 0.52)
        else:
            colors[i * n + j] = (1, 0.334, 0.52)

initialize_mesh_indices()

spring_offsets = []
if bending_springs:
    for i in range(-1, 2):
        for j in range(-1, 2):
            if (i, j) != (0, 0):
                spring_offsets.append(ti.Vector([i, j]))

else:
    for i in range(-2, 3):
        for j in range(-2, 3):
            if (i, j) != (0, 0) and abs(i) + abs(j) <= 2:
                spring_offsets.append(ti.Vector([i, j]))

@ti.kernel
def substep():
    for i in ti.grouped(x):
        v[i] += gravity * dt

    for i in ti.grouped(x):
        force = ti.Vector([0.0, 0.0, 0.0])
        for spring_offset in ti.static(spring_offsets):
            j = i + spring_offset
            if 0 <= j[0] < n and 0 <= j[1] < n:
                x_ij = x[i] - x[j]
                v_ij = v[i] - v[j]
                d = x_ij.normalized()
                current_dist = x_ij.norm()
                original_dist = quad_size * float(i - j).norm()
                # Spring force
                force += -spring_Y * d * (current_dist / original_dist - 1)
                # Dashpot damping
                force += -v_ij.dot(d) * d * dashpot_damping * quad_size

        v[i] += force * dt

    for i in ti.grouped(x):
        v[i] *= ti.exp(-drag_damping * dt)
        offset_to_center = x[i] - ball_center[0]
        if offset_to_center.norm() <= ball_radius:
            # Velocity projection
            normal = offset_to_center.normalized()
            v[i] -= min(v[i].dot(normal), 0) * normal
        x[i] += dt * v[i]

@ti.kernel
def update_vertices():
    for i, j in ti.ndrange(n, n):
        vertices[i * n + j] = x[i, j]


def draw_bounds(x_min=0, y_min=0, z_min=0, x_max=1, y_max=1, z_max=1):
    box_anchors = ti.Vector.field(3, dtype=ti.f32, shape = 8)
    box_anchors[0] = ti.Vector([x_min, y_min, z_min])
    box_anchors[1] = ti.Vector([x_min, y_max, z_min])
    box_anchors[2] = ti.Vector([x_max, y_min, z_min])
    box_anchors[3] = ti.Vector([x_max, y_max, z_min])
    box_anchors[4] = ti.Vector([x_min, y_min, z_max])
    box_anchors[5] = ti.Vector([x_min, y_max, z_max])
    box_anchors[6] = ti.Vector([x_max, y_min, z_max])
    box_anchors[7] = ti.Vector([x_max, y_max, z_max])

    box_lines_indices = ti.field(int, shape=(2 * 12))
    for i, val in enumerate([0, 1, 0, 2, 1, 3, 2, 3, 4, 5, 4, 6, 5, 7, 6, 7, 0, 4, 1, 5, 2, 6, 3, 7]):
        box_lines_indices[i] = val
    return box_anchors, box_lines_indices





box_anchors, box_lines_indices = draw_bounds(x_min=0, y_min=0, z_min=0, x_max=1, y_max=1, z_max=1)




#可视化——————————————————————————————————————————————————————————————————————————————————————————
window = ti.ui.Window("Taichi Cloth Simulation on GGUI", (1024, 1024),vsync=True)
canvas = window.get_canvas()
canvas.set_background_color((1, 1, 1))
scene = ti.ui.Scene()
camera = ti.ui.Camera()


current_t = 0.0
initialize_mass_points()

init_particles()
rotation(0)  # initially rotate the cube
compute_radius_vector()  # store the shape of rigid body
precompute_q_inv()
#

camera.position(0.0, 0.0, 10)
camera.lookat(0.0, 0.0, 0)


scene.point_light(pos=(0, 1, 2), color=(1, 1, 1))
scene.ambient_light((0.5, 0.5, 0.5))




while window.running:

    if current_t > 1.5:
        # Reset
        initialize_mass_points()
        current_t = 0

    for i in range(substeps):
        substep()
        current_t += dt
    update_vertices()
    if window.get_event(ti.ui.PRESS):
        # press space to pause the simulation
        if window.event.key == ti.ui.SPACE:
            paused[None] = not paused[None]

        # proceed once for debug
        if window.event.key == 'p':
            substep1()

    # do the simulation in each step
    if (paused[None] == False):
        for i in range(int(0.05 / dt)):
            substep1()

    # set the camera, you can move around by pressing 'wasdeq'
    camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
    scene.set_camera(camera)

    scene.mesh(vertices,
               indices=indices,
               per_vertex_color=colors,
               two_sided=True)
    # Draw a smaller ball to avoid visual penetration
    scene.particles(ball_center, radius=ball_radius * 0.95, color=(0.5, 0.42, 0.8))

    # set the light
    scene.point_light(pos=(0, 1, 2), color=(1, 1, 1))
    scene.point_light(pos=(0.5, 1.5, 0.5), color=(0.5, 0.5, 0.5))
    scene.ambient_light((0.5, 0.5, 0.5))

    scene.lines(box_anchors, indices=box_lines_indices, color=(0.99, 0.68, 0.28), width=2.0)

    # draw particles
    world_scale()
    scene.particles(pos_draw, radius=0.01, color=(0, 1, 1))

    # show the frame
    canvas.scene(scene)
    window.show()

