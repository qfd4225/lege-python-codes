#所有taichi的入口
import taichi as ti
ti.init(arch=ti.gpu)

@ti.kernel # 声明一个taichi的kernel函数 ,最外层的for会被并行
def func1():
    a=ti.Vector([0.0,0.0,0.0])
    print("a=",a)
    print("a[1]=",a[1])
    d=ti.Vector([0.0,1.0,0.0])
    print("d=",d)
    B=ti.Matrix([[1.5,1.4],[1.3,1.2]])
    print("B=",B)
    print("B[1,0]=",B[1,0])
    r=ti.Struct(v1=a,v2=d,l=1)
    print("r.v1=",r.v1)
    print("r.v2=",r.v2)
    print("r.l=",r.l)


#3D gravitational field in 256*256*128 room
gravitational_field=ti.Vector.field(n=3,dtype=ti.f32,shape=(256,256,128))
# #2d strain-tenser field in 64*64 grid
# strain_tensor_field=ti.field(n=2,m=2,dtype=ti.f32,shape=(64,64))



N=10
x=ti.field(dtype=ti.i32,shape=N)
@ti.kernel
def funcx():
    for i in range(N):
        x[i]=i
    print("x=",x[5])

y=ti.Vector.field(2,dtype=ti.i32,shape=(N,N))
@ti.kernel
def funcy():
    for i,j in y:
        #print('i:',i," j=",j)
        y[i,j]=ti.Vector([i,j])
    print("y=", y[5,5])

@ti.kernel
def func_param(x:ti.i32,y:ti.f32)->ti.f32:
    z=x + y
    print("x+y=", z)
    return  z


@ti.kernel  #强制inline，所以不支持递归
def func_combine():
    print("1")
    bar()

@ti.func
def bar():
    print("2")
    baz()
@ti.func
def baz():
    print("3")


print('func1:')
func1()

print('funcx:')
funcx()

print('funcy:')
funcy()

print('func_param:')
zz=func_param(2,3.3)#参数值不会带出来的，想带出来只能return回来
print("zz=",zz)

print('func_combine:')
func_combine()

#global scalar
global_scalar=ti.field(dtype=ti.f32,shape=())
@ti.kernel  #强制inline，所以不支持递归
def print_a():
    print("a=",global_scalar[None])

global_scalar[None]=42
print_a()
global_scalar[None]=53
print_a()

# A*B:矩阵点乘
# A@B：矩阵叉乘


# gui=ti.GUI('N-body',(512,512))
# while gui.running:
#
# for i in range(1000000):
#     print(i*0.01)
#     gui.set_image(pixels)
#     gui.show()
#