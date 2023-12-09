import csv
import numpy as np
import statistics as st
file=open("0.csv")
reader = csv.reader(file)
x = []
y = []
p = []
t = []
for row in reader:
    x.append(int(row[0]))
    y.append(int(row[1]))
    p.append(int(row[2]))
    t.append(int(row[3]))
file.close()
x = np.asarray(x)
y = np.asarray(y)
p = np.asarray(p)
t = np.asarray(t)

i = 1
period = []
while i != len(p):
    if(p[i] != p[i-1] and p[i] == 1):
        period.append(t[i] - t[i-1])
        
    i = i+1

print(st.mode(np.asarray(period)))

