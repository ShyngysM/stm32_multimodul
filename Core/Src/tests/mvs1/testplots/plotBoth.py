import pandas as pd                                         
import matplotlib.pyplot as plt

data_MCU = pd.read_csv('mvs1_MCU/mcu_data1.txt',sep=',',header=None)
df_MCU = pd.DataFrame(data_MCU)

last_x = df_MCU[0].iloc[-1]

time_mcu = (df_MCU[0]*1000/(last_x+1))
voltage_mcu = (df_MCU[1]*3.3/2**16)

# calculate time axis
    #for i in df_MCU[0]:
    #    t = i*1000/last_x
    #    time(t)
    # analogie unten

########################################################################

df = pd.read_csv('mvs1_OSZI/TEK00001.CSV', skiprows=15)

# Daten so filtern, dass es genau eine Sekunde bleibt
df_filtered = df.loc[df['CH1'] > 2.9]
#need to take last element from df_filtered and use as division frequency
time = []
#last_df_filtered = df_filtered["TIME"].iloc[-1]
# calculate time
for i in df_filtered["TIME"]:
    t = (i - df_filtered["TIME"].iloc[0])*1000
    time.append(t)

# y axis is already voltage
voltage = df_filtered["CH2"]

# plot from here
fig, ax = plt.subplots()                        

ax.plot(time, voltage, color="green", label="MVS Signal (OSZI)")
ax.plot(time_mcu,voltage_mcu, color="blue", label="MVS Signal (MCU)")
ax.set_xlabel("t in ms")
ax.set_ylabel("U in V")                             # Titel von der y-Achse
ax.legend(loc = 0)                                  # Platzierung von der Legende

plt.show()

