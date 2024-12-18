import matplotlib.pyplot as plt

# Sample data file reading and processing
filename = "comparemulti.txt"  # Replace with your file path
thread_data = {}

# Reading file
with open(filename, 'r') as file:
    current_thread = None
    for line in file:
        # If the line contains "thread", set it as the current thread
        if "thread" in line:
            current_thread = int(line.split()[0])  # Extract the number of threads
            thread_data[current_thread] = []
        else:
            # Extract the first number before the comma
            first_number = float(line.split(',')[0])
            thread_data[current_thread].append(first_number)

# Plotting
for n_threads, values in thread_data.items():
    plt.plot(values, label=f'{n_threads} threads')

plt.xlabel('# of agents')
plt.ylabel('runtime')
plt.title('# of agents vs runtime for Different Threads')
plt.legend()
plt.show()
