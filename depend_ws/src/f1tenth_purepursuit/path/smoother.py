import pandas as pd
import sys
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
from scipy.ndimage import gaussian_filter
from scipy.signal import medfilt2d
#from scipy.ndimage import gaussian_laplace

df = pd.read_csv("%s.csv"%(sys.argv[1]), header=None)
print(df)

# df = pd.concat([df, df.loc[:7]], ignore_index=True)

x, y = df.loc[:, 0], df.loc[:, 1]
xy = df.loc[:, 0:1]
#print(x)
#print(y)

sigma = int(sys.argv[2])
xp, yp = gaussian_filter1d(x, sigma), gaussian_filter1d(y, sigma)
#lp = gaussian_filter(xy, sigma)
#lp = medfilt2d(xy)
#print(lp)
#xp, yp = lp[:, 0], lp[:, 1]
#print(xp, yp)


# plt.plot(x, y)
# plt.plot(xp, yp)
# plt.show()

# replace columns
df[0] = xp
df[1] = yp

# df=df.drop(df.index[:7])

print(df)

df.to_csv("%s_smooth.csv"%(sys.argv[1]), index=False, header=False)