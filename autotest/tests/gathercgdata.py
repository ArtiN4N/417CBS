import matplotlib.pyplot as plt

data = {}

for mapn in range(1, 31):
    if mapn not in data:
        data[mapn] = {}
    for tn in [1, 2, 4, 8]:
        if tn not in data[mapn]:
            data[mapn][tn] = []

        inputFile = f'map{mapn}scen1t{tn}dg.out'

        with open(inputFile, 'r') as file:
            for line in file:
                # Extract the first number before the comma
                first_number = float(line.split(',')[0])
                data[mapn][tn].append(first_number)

#print(data)


maps = range(1, 31)  # Map numbers
speedup_2 = []
speedup_4 = []
speedup_8 = []

nagents_2 = []
nagents_4 = []
nagents_8 = []

# Compute speedup for each map
for mapn in maps:
    runtimes = data[mapn]
    avg_time_1 = sum(runtimes[1]) / len(runtimes[1])  # Average runtime for 1 thread
    avg_time_2 = sum(runtimes[2]) / len(runtimes[2])  # Average runtime for 2 threads
    avg_time_4 = sum(runtimes[4]) / len(runtimes[4])  # Average runtime for 4 threads
    avg_time_8 = sum(runtimes[8]) / len(runtimes[8])  # Average runtime for 4 threads

    nagents_2.append(sum(runtimes[2]))
    nagents_4.append(sum(runtimes[4]))
    nagents_8.append(sum(runtimes[8]))

    speedup_2.append(avg_time_1 / avg_time_2)
    speedup_4.append(avg_time_1 / avg_time_4)
    speedup_8.append(avg_time_1 / avg_time_8)


sorted_list = [x for _, x in sorted(zip(speedup_8, maps))]

print(sorted_list)  # Output: ['date', 'cherry', 'apple', 'banana']

# Plotting
plt.figure(figsize=(10, 6))

# Speedup for 2 threads
plt.plot(maps, speedup_2, label="2 Threads", marker="o", color="blue")
#plt.plot(maps, nagents_2, marker="o", color="blue")


# Speedup for 4 threads
plt.plot(maps, speedup_4, label="4 Threads", marker="s", color="green")
#plt.plot(maps, nagents_4, marker="s", color="green")

plt.plot(maps, speedup_8, label="8 Threads", marker="d", color="red")
#plt.plot(maps, nagents_8, marker="d", color="red")


# Add labels, title, and legend
plt.xlabel("Map Number")
plt.ylabel("Speedup factor")
plt.title("CG heuristic Speedup for 2 and 4 Threads Compared to 1 Thread")
plt.xticks(maps)  # Show each map number on the x-axis
plt.legend()
plt.grid(True)
plt.ylim(0, 2.5)
plt.axhline(y = 1, color="purple", linestyle="--")

# Show the plot
plt.tight_layout()
plt.show()

print(sum(speedup_2) / len(speedup_2))
print(sum(speedup_4) / len(speedup_4))
print(sum(speedup_8) / len(speedup_8))
