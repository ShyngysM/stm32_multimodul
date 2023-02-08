import pandas as pd
import matplotlib.pyplot as plt

filter = []
with open('TEK00000.CSV', 'r') as stream:
    for line in stream:
        if ",3" in line:
            filter.append(line)

df = pd.DataFrame([sub.split(",") for sub in filter])
df = df.replace('\n','', regex=True)

lx_OSZI = []

for i in df[0]:
    x = (float(i) - float(df[0].iloc[0]))*1000
    lx_OSZI.append(x)

y = df[2].astype(float)

fig, ax = plt.subplots()                        

ax.plot(lx_OSZI, y, color="green", label="MVS Signal (OSZI)")
ax.set_xlabel("t in ms")
ax.set_ylabel("U in V")                    # Titel von der y-Achse
ax.legend(loc = 0)                               # Platzierung von der Legende

plt.show()
