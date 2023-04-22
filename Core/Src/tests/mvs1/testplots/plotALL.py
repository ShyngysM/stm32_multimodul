import pandas as pd                                         
import matplotlib.pyplot as plt

# from here handling of mcu data: digital and analog
data_MCU = pd.read_csv('mvs1_MCU/data0.txt',sep=',',header=None)
df_mcu = pd.DataFrame(data_MCU)

sep_digital = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Digita")].index[0]
sep_meas = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Meas")].index[0]

analog_df = df_mcu.iloc[:sep_digital, :].astype(int) 
digital_df = df_mcu.iloc[sep_digital+1:sep_meas, :].astype(int)
meas_df =  df_mcu.iloc[sep_meas+1:, :]


last_analog = analog_df[0].iloc[-1]
time = (analog_df[0]*1000/(last_analog+1))
voltage = (analog_df[1]*3.3/2**16)



# from here handling data of OSZI
data_OSZI = pd.read_csv('mvs1_OSZI/TEK00001.CSV', skiprows=15)

# Daten so filtern, dass es genau eine Sekunde bleibt
df_filtered = data_OSZI.loc[data_OSZI['CH1'] > 2.9]

time_OSZI = []

# calculate time
for i in df_filtered["TIME"]:
    t = (i - df_filtered["TIME"].iloc[0])*1000
    time_OSZI.append(t)

# y axis is already voltage
voltage_OSZI = df_filtered["CH2"]


fig, axs = plt.subplots(3)
fig.suptitle('Vertically stacked subplots')
axs[1].plot(time, voltage)
axs[1].set_xlabel("t [ms]")
axs[1].set_ylabel("U [V]")

axs[2].plot(time, digital_df[1])
axs[2].set_xlabel("t [ms]")
axs[2].set_ylabel("U_i (0, 1)")

axs[0].plot(time_OSZI, voltage_OSZI)
axs[0].set_xlabel("t [ms]")
axs[0].set_ylabel("U [V]")

plt.show()

# import io
# f1 = ''
# f2 = ''
# f3 = ''
# with open('mvs1_MCU/data0.txt', 'r', encoding='UTF-8') as file:
#     while (line := file.readline().rstrip() and "Digital" not in line):
#         f1 += line + '/n'
#         # if "Digital" in line:
#         #     break

# f1 = io.StringIO(f1)
