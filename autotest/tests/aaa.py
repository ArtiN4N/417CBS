import matplotlib.pyplot as plt

data = {}

mapn = 24
data[0] = {}
data[1] = {}
data[2] = {}
for tn in [1, 2, 4, 8]:
    for i in [0, 1, 2]:
        if tn not in data[i]:
            data[i][tn] = []

    inputFile1 = f'map{mapn}scen1t{tn}.out'
    inputFile2 = f'map{mapn}scen1t{tn}dg.out'
    inputFile3 = f'map{mapn}scen1t{tn}wdg.out'

    with open(inputFile1, 'r') as file:
        for line in file:
            # Extract the first number before the comma
            first_number = float(line.split(',')[0])
            data[0][tn].append(first_number)
    with open(inputFile2, 'r') as file:
        for line in file:
            # Extract the first number before the comma
            first_number = float(line.split(',')[0])
            data[1][tn].append(first_number)
    with open(inputFile2, 'r') as file:
        for line in file:
            # Extract the first number before the comma
            first_number = float(line.split(',')[0])
            data[2][tn].append(first_number)

#print(data)


#maps =   # Map numbers
speedup_2 = []
speedup_4 = []
speedup_8 = []

nagents_2 = []
nagents_4 = []
nagents_8 = []

# Plotting
plt.figure(figsize=(10, 6))

colors = ["blue", "green", "red", "purple"]

i = 0
# Speedup for 2 threads
for tn in [1, 2, 4, 8]:
    plt.plot(range(len(data[0][tn])), data[0][tn], label=f"CG {tn} Threads", marker="o", color=colors[i], linestyle="--")
    plt.plot(range(len(data[1][tn])), data[1][tn], label=f"DG {tn} Threads", marker="s", color=colors[i], linestyle="--")
    plt.plot(range(len(data[2][tn])), data[2][tn], label=f"WDG {tn} Threads", marker="d", color=colors[i], linestyle="--")
    i += 1



# Add labels, title, and legend
plt.xlabel("# of agents")
plt.ylabel("Runtime")
plt.title("Heuristic runtime on map 24 compared to # of agents")
#plt.xticks(maps)  # Show each map number on the x-axis
plt.legend()
plt.yscale('log')
plt.grid(True)
plt.ylim(0, 0.5)

# Show the plot
plt.tight_layout()
plt.show()

print(sum(speedup_2) / len(speedup_2))
print(sum(speedup_4) / len(speedup_4))
print(sum(speedup_8) / len(speedup_8))
