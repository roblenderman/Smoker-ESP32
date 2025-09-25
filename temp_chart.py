import matplotlib.pyplot as plt
import re

# Read the log file
log_file = "Logs/192168122523_2025_09_20.15.40.34.693.txt"

temperatures = []
timestamps = []

with open(log_file, 'r') as f:
    lines = f.readlines()

# Extract thermocouple temperatures
for i, line in enumerate(lines):
    if '>thermocouple:' in line:
        # Extract temperature value
        match = re.search(r'>thermocouple:(\d+\.?\d*)', line)
        if match:
            temp = float(match.group(1))
            temperatures.append(temp)
            timestamps.append(i)  # Use line number as timestamp

# Create the line chart
plt.figure(figsize=(12, 6))
plt.plot(timestamps, temperatures, 'b-', linewidth=1, marker='o', markersize=2)
plt.axhline(y=250, color='r', linestyle='--', alpha=0.7, label='Setpoint (250°F)')

plt.title('Smoker Temperature Over Time', fontsize=14, fontweight='bold')
plt.xlabel('Log Entry Number', fontsize=12)
plt.ylabel('Temperature (°F)', fontsize=12)
plt.grid(True, alpha=0.3)
plt.legend()

# Add some statistics
min_temp = min(temperatures)
max_temp = max(temperatures)
avg_temp = sum(temperatures) / len(temperatures)
temp_range = max_temp - min_temp

plt.text(0.02, 0.98, f'Min: {min_temp:.1f}°F\nMax: {max_temp:.1f}°F\nRange: {temp_range:.1f}°F\nAvg: {avg_temp:.1f}°F',
         transform=plt.gca().transAxes, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

plt.tight_layout()
plt.savefig('smoker_temperature_chart.png', dpi=150, bbox_inches='tight')
plt.show()

print(f"Temperature Statistics:")
print(f"Minimum: {min_temp:.1f}°F")
print(f"Maximum: {max_temp:.1f}°F")
print(f"Range: {temp_range:.1f}°F")
print(f"Average: {avg_temp:.1f}°F")
print(f"Total readings: {len(temperatures)}")
print(f"Chart saved as 'smoker_temperature_chart.png'")