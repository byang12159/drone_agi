#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from datetime import datetime

# CSV file path
csv_file = 'data.csv'

# Lists to store data
times = []
x_values = []

# Read data from CSV
with open(csv_file, 'r', newline='') as f:  # Open the file in text mode for Python 3
    reader = csv.reader(f)
    next(reader)  # Skip header row if present
    for row in reader:
        timestamp = float(row[0])  # Assuming first column is Unix timestamp
        times.append(datetime.utcfromtimestamp(timestamp))  # Convert Unix timestamp to datetime object
        x_values.append(float(row[1]))  # Assuming second column is x-values

# Plot the data
plt.plot(times, x_values)
plt.xlabel('Time')
plt.ylabel('X Value')
plt.title('X Value vs. Time')
plt.xticks(rotation=45)  # Rotate x-axis labels for better readability
plt.grid(True)
plt.tight_layout()  # Adjust layout to prevent clipping of labels
plt.show()

