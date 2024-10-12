#! /usr/bin/python
# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt


position = np.genfromtxt('cart_position.txt')
# force_base = np.genfromtxt('ext_force_base.txt')

plt.subplot(3, 3, 1)
plt.plot(position[:, 0])
plt.title("Position X")

plt.subplot(3, 3, 2)
plt.plot(position[:, 1])
plt.title("Position Y")

plt.subplot(3, 3, 3)
plt.plot(position[:, 2])
plt.title("Position Z")


# plt.subplot(3, 3, 7)
# plt.plot(force_base[:, 0])
# plt.title("force_base X")

# plt.subplot(3, 3, 8)
# plt.plot(force_base[:, 1])
# plt.title("force_base Y")

# plt.subplot(3, 3, 9)
# plt.plot(force_base[:, 2])
# plt.title("force_base Z")
plt.show()
