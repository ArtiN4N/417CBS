import matplotlib.pyplot as plt

data = {}

for mapn in range(1, 31):
    if mapn not in data:
        data[mapn] = {}
    for tn in [1, 2, 4, 8]:
        if tn not in data[mapn]:
            data[mapn][tn] = []

        inputFile = f'map{mapn}scen1t{tn}.out'

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

# Compute speedup for each map
for mapn in maps:
    runtimes = data[mapn]
    avg_time_1 = sum(runtimes[1]) / len(runtimes[1])  # Average runtime for 1 thread
    avg_time_2 = sum(runtimes[2]) / len(runtimes[2])  # Average runtime for 2 threads
    avg_time_4 = sum(runtimes[4]) / len(runtimes[4])  # Average runtime for 4 threads
    avg_time_8 = sum(runtimes[8]) / len(runtimes[8])  # Average runtime for 4 threads

    speedup_2.append(avg_time_1 / avg_time_2)
    speedup_4.append(avg_time_1 / avg_time_4)
    speedup_8.append(avg_time_1 / avg_time_8)


# Plotting
plt.figure(figsize=(10, 6))

# Speedup for 2 threads
plt.plot(maps, speedup_2, label="2 Threads", marker="o", color="blue")

# Speedup for 4 threads
plt.plot(maps, speedup_4, label="4 Threads", marker="s", color="green")
plt.plot(maps, speedup_8, label="8 Threads", marker="d", color="red")

# Add labels, title, and legend
plt.xlabel("Map Number")
plt.ylabel("Speedup")
plt.title("Speedup for 2 and 4 Threads Compared to 1 Thread")
plt.xticks(maps)  # Show each map number on the x-axis
plt.legend()
plt.grid(True)
plt.ylim(0, 2)
plt.axhline(y = 1, color="purple", linestyle="--")

# Show the plot
plt.tight_layout()
plt.show()
