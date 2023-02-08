import pandas as pd                                         
import matplotlib.pyplot as plt

data_MCU = pd.read_csv('data0.txt',sep=',',header=None)
df_MCU = pd.DataFrame(data_MCU)

lx = []
ly = df_MCU[1]
last_x = df_MCU[0].iloc[-1]
last_y = df_MCU[1].iloc[-1]

# calculate time axis
for i in df_MCU[0]:
    x = i*1000/last_x
    lx.append(x)

#for i in df_MCU[1]
#    y = 

# plot from here
fig, ax = plt.subplots()                        

ax.plot(lx, ly, color="blue", label="MVS Signal (µCU)")
ax.set_xlabel("t in ms")
ax.set_ylabel("U in V")                                     
ax.legend(loc = 0)                                         

plt.show()
