import pandas as pd                                         
import matplotlib.pyplot as plt
import numpy as np

data_MCU = pd.read_csv('mvs1_MCU/mcu_data1.txt',sep=',',header=None)
df_MCU = pd.DataFrame(data_MCU)

last_x = df_MCU[0].iloc[-1]
time = (df_MCU[0]*1000/last_x)
voltage = (df_MCU[1]*3.3/2**16)
pulses = []

#for u in voltage:
#    if u > 2:
#        pulses.append(1)
#    else:
#        pulses.append(0)

#list comprehension
#pulses = [1 if u > 2 else 0 for u in voltage]

#numpy get boolean, transfer to int
#pulse = voltage > 2
#pulse = pulse.astype(int)

#numpy query
pulse = np.where(voltage > 2, 1, 0)
# calculate time axis
    #for i in df_MCU[0]:
    #    t = i*1000/last_x
    #    time(t)
    # analogie unten

# plot from here
fig, ax = plt.subplots()                        

ax.plot(time, pulse, color="blue", label="MVS Signal (µCU)")
ax.set_xlabel("t in ms")
ax.set_ylabel("U in V")                                     
ax.legend(loc = 0)                                         

plt.show()
