import pandas as pd                                         
import matplotlib.pyplot as plt

# scaling the time axis 1000 = ms
SCALE = 1000
ADC_RES = 2**16
REF_VOLTAGE = 3.3

#### DATA PREPARATION #### 
data_MCU = pd.read_csv('dataMCU/newtrig1.txt',sep=',',header=None)
data_OSZI = pd.read_csv('dataOSZI/TEKtrig.CSV', skiprows=15)           

df_mcu = pd.DataFrame(data_MCU)                                        
# filtering data: oczi is triggered on rising edge CH1, so the sample time is undefined.
# that's why we have an extra signal, which is HIGH while the measurement process
df_oszi = data_OSZI.loc[data_OSZI['CH1'] > 2.9]

import numpy as np
test = df_oszi.index[1:] - df_oszi.index[:-1]
edges = test[test > 1]
if len(edges)> 1:
   print('found more than one gap', edges)
edge = edges[0]
df_oszi = df_oszi.loc[edge:]

#reset indices: don't need them after this line
df_oszi.reset_index(drop=True, inplace=True)

# create separators to split the dataframe by keywords
sep_digital = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Digita")].index[0]
sep_meas = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Meas")].index[0]

# split dataframes
analog_df = df_mcu.iloc[:sep_digital, :].astype(int) 
digital_df = df_mcu.iloc[sep_digital+1:sep_meas, :].astype(int)
meas_df =  df_mcu.iloc[sep_meas+1:, :]

# dummy concatenation of rows with measurements (possible only cause they are strings (objects))
info = meas_df.iloc[0,0] + "\n" + meas_df.iloc[1,0]

# resolving axis from row integers to human readable data µC
 
time = analog_df.index/len(analog_df)
voltage = (analog_df[1]*REF_VOLTAGE/ADC_RES)
# for µC digitalized not needed

# offset the time for oszi signal
time_OSZI = (df_oszi["TIME"] - df_oszi["TIME"].iloc[0])


####logging####

#first time oszi read
first_time_oszi = df_oszi.iloc[0]
last_time_oszi = df_oszi.iloc[-1]
print("oszi_recorded_time", last_time_oszi["TIME"] - first_time_oszi["TIME"])

#sensor started vibrating
sensor_oszi = df_oszi.loc[df_oszi["CH2"]<0.4]

#first vibration point
first_pulse_oszi = sensor_oszi[["CH2","TIME"]].iloc[0]
last_pulse_oszi = sensor_oszi[["CH2","TIME"]].iloc[-1]

#gap between first read and first vibration
gap_oszi = first_pulse_oszi["TIME"] - first_time_oszi["TIME"]
print("gap_oszi: ", first_pulse_oszi["TIME"] - first_time_oszi["TIME"])

#period of vibration from oszi
print("vibration_time_oszi: ", last_pulse_oszi["TIME"] - first_pulse_oszi["TIME"])
#period of vibration from oszi based on its indices
print("vibration_index_time_oszi: ", (last_pulse_oszi.name - first_pulse_oszi.name)/len(df_oszi))
#print(first_time_oszi.name)

index_gap_oszi = (first_pulse_oszi.name - first_time_oszi.name)/len(df_oszi)

#period of vibration from oszi based on indices not TIME channel
print("index_gap_oszi: ", index_gap_oszi)

#logging for mcu
#MCU only have indices no time chanel
#sensor vibration by MCU
sensor_voltage = voltage.loc[voltage < 0.4]

#gap between first read and first vibration on MCU
index_gap_mcu = sensor_voltage.index[0]/len(voltage)
print("index_gap_mcu: ", index_gap_mcu)

print("difference between oszi first read and MCU: ", index_gap_mcu - index_gap_oszi)
diff_gap_oszi_mcu = index_gap_mcu - index_gap_oszi

#vibration period by MCU
print("vibration_index_time_mcu: ", (sensor_voltage.index[-1] - sensor_voltage.index[0])/len(voltage))

### end logging ####


print(df_oszi["TIME"].iloc[0] - df_oszi["TIME"].iloc[-1])
print(len(df_oszi))
# y axis is already voltage
voltage_OSZI = df_oszi["CH2"]

#### Plots #### 

fig, axs = plt.subplots(5)
fig.suptitle("Comparison of sampled data: OSZI vs µC")

axs[0].plot(data_OSZI["TIME"]*SCALE, data_OSZI["CH1"], color="orange", label="Help signal")
axs[0].plot(data_OSZI["TIME"]*SCALE, data_OSZI["CH2"], color="green", label="MVS Signal OSZI")
axs[0].set_ylabel("U [V]")
axs[0].set_xlabel("t [ms]")

axs[1].plot(time_OSZI*SCALE, voltage_OSZI, color="green", label="MVS Signal OSZI")
axs[1].set_ylabel("U [V]")
axs[1].set_xlabel("t [ms]")

axs[2].plot(time * SCALE, voltage, color="red", label="MVS Signal µC")
axs[2].set_ylabel("U [V]")
axs[2].set_xlabel("t [ms]")

my_yticks = [0,1]
axs[3].set_yticks(my_yticks)
axs[3].plot(time * SCALE, digital_df[1],  color="blue", label="MVS Signal \n µC - discretisation")
axs[3].set_ylabel("U_i (0, 1)")

axs[4].plot(time_OSZI * SCALE, voltage_OSZI, color="green", label="MVS Signal OSZI \n overlay")
axs[4].plot((time - diff_gap_oszi_mcu) * SCALE, voltage, color="red", label="MVS Signal µC")
axs[4].set_ylabel("U [V]")
axs[4].set_xlabel("t [ms]")

axs[0].legend(loc = 0)
axs[1].legend(loc = 0)
axs[2].legend(loc = 0)
axs[3].legend(loc = 0)
axs[4].legend(loc = 0)

plt.figtext(0.5, 0.9, info, ha="center", fontsize=7, bbox={"facecolor":"orange", "alpha":0.5, "pad":5})

plt.show()
