import numpy as np
import matplotlib.pyplot as plt

# 定义x值的范围
x = np.linspace(0.001, 1, 1000)

# 计算对应的y值
y = -20 * np.log(x)

# 使用pyplot来绘制函数曲线
plt.figure(figsize=(8, 5))
plt.plot(x, y, label=r'$-20 \times \log(X)$')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot of -20 * log(X)')
plt.grid(True)

# 标出指定的点
points_x = [0.001, 0.01, 0.05, 0.1, 0.2, 1]
points_y = [-20 * np.log(xx) for xx in points_x]

for xx, yy in zip(points_x, points_y):
    plt.annotate(f'({xx:.3f}, {yy:.3f})', xy=(xx, yy), xytext=(xx + 0.01, yy - 5),
                 arrowprops=dict(facecolor='black', shrink=0.05))
    plt.scatter(xx, yy, color='red')

plt.legend()
plt.show()