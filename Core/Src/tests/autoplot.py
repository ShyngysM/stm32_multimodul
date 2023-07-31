import pandas as pd                                         
import matplotlib.pyplot as plt
import numpy as np

# scaling the time axis 1000 = 1 ms
SCALE = 1000
ADC_RES = 2**16
REF_VOLTAGE = 3.3
signals = []
measurements = []
FILE = 'Kupferkugel_Dauertest/mvs5_dt.txt'


#### DATA PREPARATION #### 
raw_data = pd.read_csv(FILE ,sep=',',header=None, skiprows=1)          
df = pd.DataFrame(raw_data)                                        

# create separators to split the dataframe by keywords
seps_meas = df[df.astype(str).iloc[:,0].str.contains("Meas")]
seps_numb = df[df.astype(str).iloc[:,0].str.contains("Numb")]

mvs1_sig = df.iloc[:seps_meas.index[0],:].astype(int)
mvs10_meas = df.iloc[seps_meas.index[9]+1: ,:]

for i in range(len(seps_numb.index)):
    mvs_sig = df.iloc[seps_numb.index[i]+1:seps_meas.index[i+1],:].astype(int)
    signals.append(mvs_sig)
    mvs_meas = df.iloc[seps_meas.index[i]+1: seps_numb.index[i],:]
    measurements.append(mvs_meas)

signals.insert(0, mvs1_sig)
measurements.append(mvs10_meas)

#### LOGS ####
seps_bad = df[df.astype(str).iloc[:,0].str.contains("bad")]
print(seps_bad)

#### PLOTS ####
for i in range(len(seps_numb.index)+1):
# a bit of calculations, to have a human readable values
    analog_df = signals[i]
    last_analog = df.iloc[seps_meas.index[i]-1].astype(int)
    time = (analog_df[0]*SCALE/(last_analog[0]+1))
    voltage = (analog_df[1]*REF_VOLTAGE/ADC_RES)

    meas_df = measurements[i]
    info = meas_df.iloc[0,0] + "\n" + meas_df.iloc[1,0]

    fig, axs = plt.subplots(1,1)
    fig.suptitle(FILE + "\n" + "Measurement " + str(i+1))
    axs.plot(time, voltage, color="blue", label="Measurements:" + "\n" + info)
    axs.set_xlabel("t in [ms]")
    axs.set_ylabel("U in [V]")
    axs.legend(loc=4)

plt.show()
