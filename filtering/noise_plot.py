#
#
#

import sys
sys.path.insert(0, '../lib')

import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt

from data.plot import *
from imu_driver import *

drv = IMUDriver()
drv.open()
plot = DataPlotter()

for i in range(0, 1000):
    (ax, ay, az, gx, gy, gz) = drv.sample()
    plot.add('i', i)
    plot.add('gz', gz)

gz_array = plot.data['gz']
avg = np.mean(gz_array)
for i in range(0, 1000):
    plot.add('avg', avg)

plot.plot( ['i', 'Samples'], [ [ 'gz', 'Gyro Z' ], ['avg', 'Average' ] ])
plot.show()

plt.hist(gz_array,bins=100,density=True,label="Histogram of samples")
plt.show()

