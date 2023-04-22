# Notwendige Bibliotheken ("as" - sehe Kommentar auf Zeile 2!)
import matplotlib.pyplot as plt                 # Wir wollen nicht immer "matplotlib.pyplot" schreiben, deswegen ist es mit "as" umbenannt!
import numpy as np                           



# Definition von 2-dimensionalen Raum (von, bis, in 1000 Schritten)
x = np.linspace(1, 10000, 100000)         

L1_2 = 10*10**(-3)
C = 100*10**(-9)
# Definition von mathematischen Funktionen:
y1 = 1/(L1_2/(L1_2-1/x**2) + 1)                    
y2 = 1/(1/(x**2 * L1_2 * C - 1) + 1) 

# Definition von einen "Rechteck" Funktion:
def rect(x):
    return np.where (abs(x)<=2, 1, 0)





# Definiere eine Figure mit mehreren subplots drauf:
fig, ax = plt.subplots()                        

# Erzeugung von Funktionen:
ax.plot(x, y1, color="blue", label="Funktion 1")  
ax.plot(x, y2, color="red", label="Funktion 2")      
# ax.plot(x, rect(x), color="green", label="Rechteck_fkt.")   
ax.set_xscale("log")
ax.set_xlabel("Drehmoment")                      # Titel von der x-Achse 
ax.set_ylabel("Polradwinkel")                    # Titel von der y-Achse
ax.legend(loc = 0)                               # Platzierung von der Legende

# Zeige was kompiliert wurde:
plt.show()                                       


