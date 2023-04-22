import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('mvs1_OSZI/TEK00001.CSV', skiprows=15)

# Daten so filtern, dass es genau eine Sekunde bleibt
df_filtered = df.loc[df['CH1'] > 2.9]

time = []

# calculate time
for i in df_filtered["TIME"]:
    t = (i - df_filtered["TIME"].iloc[0])*1000
    time.append(t)

# y axis is already voltage
voltage = df_filtered["CH2"]

# plot from here
fig, ax = plt.subplots()                        

ax.plot(time, voltage, color="green", label="MVS Signal (OSZI)")
ax.set_xlabel("t in ms")
ax.set_ylabel("U in V")                             # Titel von der y-Achse
ax.legend(loc = 0)                                  # Platzierung von der Legende

plt.show()
