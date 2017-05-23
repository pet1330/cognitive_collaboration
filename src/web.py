import matplotlib.pyplot as plt, numpy as np, mpld3


np.random.seed(123456789)

xx = np.random.laplace(size=100)
yy = np.random.laplace(size=100)
ss = map(lambda x: 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'[x], np.random.randint(26, size=100))

plt.plot(xx, yy, linestyle='none', marker='s', mew=2, mec='k', mfc='none')
for s_i, x_i, y_i in zip(ss, xx, yy):
    plt.text(x_i+.1, y_i, s_i, ha='left', va='center')

plt.show()