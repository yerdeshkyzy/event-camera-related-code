import numpy as np
import csv
from matplotlib import pyplot as plt 

file = open("2024-03-16-09-05-37.csv")
csvreader = csv.reader(file)
n = []
for row in csvreader:
    if (row[0]):
        n.append(int(row[0]))
    
    

file.close()
n = np.asarray(n)


i=1
j=0
sum_arr = []
print(len(n))
num_loops = n[len(n)-1]//100

while i < num_loops:
    sum=0
    while n[j] < 100*i:
        sum = sum + n[j]
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

    