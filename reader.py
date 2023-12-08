import numpy as np
import csv
from matplotlib import pyplot as plt 

file = open("0.csv")
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
num_loops = t[len(t)-1]//50     #the last number here is the no of microsec for packaging events together

while i < num_loops:
    sum=0
    while t[j] < 50*i:
        sum = sum + 1#p[j]
        j = j+1
    sum_arr.append(sum)
    i = i+1


newarr = np.array_split(sum_arr, 140)
time_axis =50 * np.arange(1, len(newarr[9])+1) /1000   # the first number here is the number mentioned above


plt.title("Graph") 
plt.xlabel("time in milliseconds") 
plt.ylabel("number of positive events") 
plt.plot(time_axis,newarr[9],'.') 
plt.show() 

    