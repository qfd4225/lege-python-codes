#所有taichi的入口
import taichi as ti
ti.init(arch=ti.gpu)

@ti.kernel # 声明一个taichi的kernel函数
def func1():
    print('hello')

#太极支持类型
#有符号整形:ti.i8 ti.i16 ti.i32 ti.i64
#无符号整形:ti.u8 ti.u16 ti.u32 ti.u64
#浮点型:ti.f32 ti.f64
#ti.init(default_fp=ti.f32)
#ti.init(default_fp=ti.f64)
#ti.init(default_ip=ti.i32)
#ti.init(default_ip=ti.i64)

@ti.kernel
def func2():
    a = 1.7
    b = ti.cast(a, ti.i32)
    c = ti.cast(b, ti.f32)
    print("b =", b,end="        ") # b = 1
    print("c =", c) # c = 1.0

print('func1:')
func1()
print('func2:')
func2()


n = 320
pixels = ti.field(dtype=float, shape=(n * 2, n))

@ti.func
def complex_sqr(z):
    return ti.Vector([z[0]**2 - z[1]**2, z[1] * z[0] * 2])

@ti.kernel
def paint(t: float):
    for i, j in pixels:  # Parallelized over all pixels
        c = ti.Vector([-0.8, ti.cos(t) * 0.2])
        z = ti.Vector([i / n - 1, j / n - 0.5]) * 2
        iterations = 0
        while z.norm() < 20 and iterations < 50:
            z = complex_sqr(z) + c
            iterations += 1
        pixels[i, j] = 1 - iterations * 0.02

gui = ti.GUI("Hello World", res=(n * 2, n))

for i in range(1000000):
    paint(i * 0.03)
    gui.set_image(pixels)
    gui.show()
