import pandas as pd                                         
import matplotlib.pyplot as plt
import numpy as np

# scaling the time axis 1000 = 1 ms
SCALE = 1000
ADC_RES = 2**16
REF_VOLTAGE = 3.3

#### DATA PREPARATION #### 
data_MCU = pd.read_csv('Stahlkugel/mvs1.txt',sep=',',header=None)          

df_mcu = pd.DataFrame(data_MCU)                                        

# create separators to split the dataframe by keywords
sep_digital = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Digita")].index[0]
sep_meas = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Meas")].index[0]

# split dataframes
analog_df = df_mcu.iloc[:sep_digital, :].astype(int) 
# digital_df = df_mcu.iloc[sep_digital+1:sep_meas, :].astype(int)
meas_df =  df_mcu.iloc[sep_meas+1:, :]

# dummy concatenation of rows with measurements (possible only cause they are strings (objects))
info = meas_df.iloc[0,0] + "\n" + meas_df.iloc[1,0]

# resolving axis from row integers to human readable data
# µCU
last_analog = analog_df[0].iloc[-1]
time = (analog_df[0]*SCALE/(last_analog+1))
voltage = (analog_df[1]*REF_VOLTAGE/ADC_RES)


#### PLOTS #### 

fig, axs = plt.subplots(3)
fig.suptitle("Comparison of sampled data: OSZI vs µCU")

axs[0].plot(time_OSZI, voltage_OSZI, color="green", label="MVS Signal (OSZI)")
axs[0].set_ylabel("U [V]")
axs[0].plot(time, voltage, color="red", label="MVS Signal (µCU)")
axs[0].set_ylabel("U [V]")

axs[1].plot(time, voltage, color="red", label="MVS Signal (µCU)")
axs[1].set_ylabel("U [V]")

# my_yticks = [0,1]
# axs[2].set_yticks(my_yticks)
# axs[2].plot(time, digital_df[1], color="blue", label="MVS Signal \n (µCU - discretisation)")
# axs[2].set_xlabel("t [ms]")
# axs[2].set_ylabel("U_i (0, 1)")

# Platzierung von der Legende
axs[0].legend(loc = 0)                                                     
axs[1].legend(loc = 0)                                  
# axs[2].legend(loc = 0)                                  

plt.figtext(0.7, 0.17, info, ha="center", fontsize=7, bbox={"facecolor":"orange", "alpha":0.5, "pad":5})
plt.show()
