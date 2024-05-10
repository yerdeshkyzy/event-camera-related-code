import numpy as np
import csv
from matplotlib import pyplot as plt 

file = open("14.csv")
csvreader = csv.reader(file)
x = []
y = []
p = []
t = []
for row in csvreader:
    x.append(int(row[0]))
    y.append(int(row[1]))
    p.append(int(row[2]))
    t.append(int(row[3]))
file.close()
x = np.asarray(x)
y = np.asarray(y)
p = np.asarray(p)
t = np.asarray(t)
t = t - t[0]

x0 = min(x)
y0 = min(y)
x1 = max(x)
y1 = max(y)
i=1
j=0
sum_arr = []
print(len(t))
num_loops = t[len(t)-1]//100

while i < num_loops:
    sum=0
    while t[j] < 100*i:
        sum = sum + p[j]
        j = j+1
    sum_arr.append(sum)
    i = i+1


newarr = np.array_split(sum_arr, 20)
time_axis = 100 * np.arange(1, len(newarr[0])+1) /1000

plt.title("Graph") 
plt.xlabel("time in milliseconds") 
plt.ylabel("number of positive events") 
plt.plot(time_axis,newarr[0],'.') 
plt.show() 

    