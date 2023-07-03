import pandas as pd                                         
import matplotlib.pyplot as plt
import numpy as np

# scaling the time axis 1000 = 1 ms
SCALE = 1000
ADC_RES = 2**16
REF_VOLTAGE = 3.3
sig_arr = []
meas_arr = []
fig = []


#### DATA PREPARATION #### 
data_MCU = pd.read_csv('Stahlkugel/mvs1.csv',sep=',',header=None, skiprows=1)          

df_mcu = pd.DataFrame(data_MCU)                                        

# create separators to split the dataframe by keywords

seps_meas = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Meas")]
seps_numb = df_mcu[df_mcu.astype(str).iloc[:,0].str.contains("Numb")]

# TODO
# delete all Number cause it work even without it:)
# try it with regex in python, mb but if no time just with vim regex

mvs1_sig = df_mcu.iloc[:seps_meas.index[0],:].astype(int)
mvs10_meas = df_mcu.iloc[seps_meas.index[9]+1: ,:]

for i in range(len(seps_numb.index)):
    # TODO Insertion google it
    # sig_arr[0] = mvs1_sig
    mvs_sig = df_mcu.iloc[seps_numb.index[i]+1:seps_meas.index[i+1],:].astype(int)
    sig_arr.append(mvs_sig)
    mvs_meas = df_mcu.iloc[seps_meas.index[i]+1: seps_numb.index[i],:]
    meas_arr.append(mvs_meas)

for i in range(len(seps_numb.index)):
    analog_df = sig_arr[i]
    last_analog = df_mcu.iloc[seps_meas.index[i]-1].astype(int)
    time = (analog_df[0]*SCALE/(last_analog[0]+1))
    voltage = (analog_df[1]*REF_VOLTAGE/ADC_RES)

    meas_df = meas_arr[i]
    info = meas_df.iloc[0,0] + "\n" + meas_df.iloc[1,0]
    # plt.figtext(0.7, 0.17, info, ha="center", fontsize=7, bbox={"facecolor":"orange", "alpha":0.5, "pad":5})
    fig, axs = plt.subplots(1,1)
    axs.plot(time, voltage, color="blue", label="plot"+ i+1 + "meas" + i + "\n" +info)
    axs.set_xlabel("t in [ms]")
    axs.set_ylabel("U in [V]")
    axs.legend(loc=4)

plt.show()
