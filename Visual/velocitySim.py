from matplotlib import pyplot as plt

fps = 30
dt = 1/fps
t_current = 0
t_list = []
x_target = 10
x_start = 0
x_current = 0
x_mid = (x_target+x_start)/2
v_current = 0
x_list = []
v_list = []

def sign(x):
    if x < 0:
        return -1
    else:
        return 1


while(t_current<10):
    v_current = sign(x_target-x_current)*(abs(x_mid)-abs(x_mid-x_current)+0.1)
    x_current += v_current*dt
    t_current += dt
    x_list.append(x_current)
    v_list.append(v_current)
    t_list.append(t_current)

plt.plot(t_list, x_list, label = 'position')
plt.plot(t_list, v_list, label = 'velocity')
plt.legend()
plt.show()


    