import tkinter as tk
from tkinter import ttk
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import time

# Initialize global variables
ser = None
running = False
initialized = False
rolling_window_active = False
actual_values_window_active = False
window_size = 50

# Initialize values_labels variable
values_labels = None
baud_rate = 115200
serial_port = '/dev/ttyACM0'

# Lists to store all data
all_x_data = []
all_y_data_primary = []
all_y_data_secondary = []
all_y_data_tertiary = []

# Rolling window data
rolling_x_data = []
rolling_y_data_primary = []
rolling_y_data_secondary = []
rolling_y_data_tertiary = []

def read_serial_and_plot():
    global running, initialized, ser
    while running:
        try:
            if ser and ser.is_open:  # Check if serial connection is open
                # Read a line from the serial port
                data = ser.readline().decode().strip()
                if data:
                    # Split the data into individual label-value pairs
                    pairs = data.split('\t')

                    # Initialize lists to store values for different groups
                    group1_labels = []
                    group1_values = []
                    group2_labels = []
                    group2_values = []
                    group3_labels = []
                    group3_values = []

                    # Loop through each label-value pair
                    for pair in pairs:
                        try:
                            label, value = pair.split(':')
                            label = label.strip()
                            value = float(value.strip())
                        except ValueError:
                            continue  # Skip if there's an error

                        # Check the starting character of the label
                        if label[0] in ['D', 'R']:
                            group1_labels.append(label)
                            group1_values.append(value)
                        elif label[0] == 'M':
                            group2_labels.append(label)
                            group2_values.append(value)
                        elif label[0] == 'E':
                            group3_labels.append(label)
                            group3_values.append(value)

                    # Combine labels and values for each group
                    group1_data = list(zip(group1_labels, group1_values))
                    group2_data = list(zip(group2_labels, group2_values))
                    group3_data = list(zip(group3_labels, group3_values))

                    if not initialized:
                        initialize_plot_lines(len(group1_data), len(group2_data), len(group3_data), group1_labels, group2_labels, group3_labels)
                        initialized = True

                    # Update the plot
                    update_plot(group1_data, group2_data, group3_data)

                    # Update the actual values window if active
                    if actual_values_window_active:
                        update_actual_values_window(group1_data, group2_data, group3_data)
            else:
                print("Serial connection is not open")
                time.sleep(1)  # Wait for 1 second before retrying

        except Exception as e:
            print("Error processing serial data:", e)

def update_plot(group1_data, group2_data, group3_data):
    global all_x_data, rolling_x_data, all_y_data_primary, rolling_y_data_primary, all_y_data_secondary, rolling_y_data_secondary, all_y_data_tertiary, rolling_y_data_tertiary

    all_x_data.append(len(all_x_data))

    # Ensure y_data lists are long enough to store all data
    for i, (label, value) in enumerate(group1_data):
        if i >= len(all_y_data_primary):
            all_y_data_primary.append([])
        all_y_data_primary[i].append(value)

    for i, (label, value) in enumerate(group2_data):
        if i >= len(all_y_data_secondary):
            all_y_data_secondary.append([])
        all_y_data_secondary[i].append(value)

    for i, (label, value) in enumerate(group3_data):
        if i >= len(all_y_data_tertiary):
            all_y_data_tertiary.append([])
        all_y_data_tertiary[i].append(value)

    if rolling_window_active:
        rolling_x_data = all_x_data[-window_size:]
        for i in range(len(all_y_data_primary)):
            if i >= len(rolling_y_data_primary):
                rolling_y_data_primary.append([])
            rolling_y_data_primary[i] = all_y_data_primary[i][-window_size:]
        for i in range(len(all_y_data_secondary)):
            if i >= len(rolling_y_data_secondary):
                rolling_y_data_secondary.append([])
            rolling_y_data_secondary[i] = all_y_data_secondary[i][-window_size:]
        for i in range(len(all_y_data_tertiary)):
            if i >= len(rolling_y_data_tertiary):
                rolling_y_data_tertiary.append([])
            rolling_y_data_tertiary[i] = all_y_data_tertiary[i][-window_size:]
    else:
        rolling_x_data = all_x_data
        rolling_y_data_primary = all_y_data_primary
        rolling_y_data_secondary = all_y_data_secondary
        rolling_y_data_tertiary = all_y_data_tertiary

    for i, (label, value) in enumerate(group1_data):
        if i < len(lines_primary):
            lines_primary[i].set_data(rolling_x_data, rolling_y_data_primary[i])

    for i, (label, value) in enumerate(group2_data):
        if i < len(lines_secondary):
            lines_secondary[i].set_data(rolling_x_data, rolling_y_data_secondary[i])

    for i, (label, value) in enumerate(group3_data):
        if i < len(lines_tertiary):
            lines_tertiary[i].set_data(rolling_x_data, rolling_y_data_tertiary[i])

    plot_primary.relim()
    plot_primary.autoscale_view()
    plot_secondary.relim()
    plot_secondary.autoscale_view()
    plot_tertiary.relim()
    plot_tertiary.autoscale_view()
    plot_canvas.draw()

def clear_plot():
    # Clear the existing plot data
    for line in lines_primary:
        line.set_data([], [])
    for line in lines_secondary:
        line.set_data([], [])
    for line in lines_tertiary:
        line.set_data([], [])
    
    # Update the plot
    plot_canvas.draw()

def clear_actual_values_window():
    global values_labels
    if values_labels:
        for label in values_labels:
            label.config(text="")

def clear_plot_labels():
    global lines_primary, lines_secondary, lines_tertiary
    for line in lines_primary:
        line.set_label('')
    for line in lines_secondary:
        line.set_label('')
    for line in lines_tertiary:
        line.set_label('')
    plot_primary.legend(loc='upper left')
    plot_secondary.legend(loc='upper left')
    plot_tertiary.legend(loc='upper left')
    plot_canvas.draw()

def start_serial_connection():
    global ser, running, initialized, all_x_data, all_y_data_primary, all_y_data_secondary, all_y_data_tertiary, baud_rate, serial_port
    try:
        ser = serial.Serial(serial_port, baud_rate)  # Adjust to the correct port and baudrate
        running = True
        initialized = False
        all_x_data = []
        all_y_data_primary = []
        all_y_data_secondary = []
        all_y_data_tertiary = []
        
        # Clear the existing plot data and labels
        clear_plot()
        clear_actual_values_window()
        clear_plot_labels()
        
        status_bar.config(text=f"Connected to {serial_port} at {baud_rate} baud")
        print("Serial connection started")
        
        # Start the thread to read and plot data
        threading.Thread(target=read_serial_and_plot, daemon=True).start()
    except Exception as e:
        status_bar.config(text=f"Error: {e}")
        print("Error starting serial connection:", e)

# Function to stop the serial connection
def stop_serial_connection():
    global ser, running
    running = False
    if ser and ser.is_open:
        ser.close()
        status_bar.config(text="Serial connection stopped")
        print("Serial connection stopped")
    else:
        status_bar.config(text="Serial connection is not open")
        print("Serial connection is not open")

# Function to toggle the rolling window
def toggle_rolling_window():
    global rolling_window_active, all_x_data, rolling_x_data, all_y_data_primary, rolling_y_data_primary, all_y_data_secondary, rolling_y_data_secondary, all_y_data_tertiary, rolling_y_data_tertiary
    rolling_window_active = not rolling_window_active
    if rolling_window_active:
        rolling_button.config(text="Disable Rolling Window")
        rolling_x_data = all_x_data[-window_size:]
        for i in range(len(all_y_data_primary)):
            rolling_y_data_primary[i] = all_y_data_primary[i][-window_size:]
        for i in range(len(all_y_data_secondary)):
            rolling_y_data_secondary[i] = all_y_data_secondary[i][-window_size:]
        for i in range(len(all_y_data_tertiary)):
            rolling_y_data_tertiary[i] = all_y_data_tertiary[i][-window_size:]
    else:
        rolling_button.config(text="Enable Rolling Window")
        rolling_x_data = all_x_data
        rolling_y_data_primary = all_y_data_primary
        rolling_y_data_secondary = all_y_data_secondary
        rolling_y_data_tertiary = all_y_data_tertiary

    # Redraw the plot with the updated data
    update_plot([], [], [])

    print(f"Rolling window {'enabled' if rolling_window_active else 'disabled'}")

# Function to create and update the actual values window
def create_actual_values_window():
    global values_labels, actual_values_window_active
    actual_values_window = tk.Toplevel(root)
    actual_values_window.title("Actual Values")
    actual_values_window.geometry("200x400")  # Set the size of the window
    actual_values_window_active = True
    values_labels = []
    for i in range(len(all_y_data_primary) + len(all_y_data_secondary) + len(all_y_data_tertiary)):
        label_text = f"Value {i + 1}: "
        label = ttk.Label(actual_values_window, text=label_text, anchor='e', width=30)
        label.pack(anchor=tk.W, padx=10, pady=2)
        values_labels.append(label)

def update_actual_values_window(group1_data, group2_data, group3_data):
    global values_labels
    combined_data = group1_data + group2_data + group3_data
    for i, (label, value) in enumerate(combined_data):
        # Format the value with fixed width and zero decimal places
        formatted_value = "{:=+8.0f}".format(value)
        values_labels[i].config(text=f"{label}: {formatted_value}")

# Tkinter GUI
root = tk.Tk()
root.title("Live Timeseries Plot")

# Create the plot
figure = Figure(figsize=(10, 8), dpi=100)

# Primary y-axis plot (for small values)
plot_primary = figure.add_subplot(3, 1, 1)
# plot_primary.set_xlabel('Time')
plot_primary.set_ylabel('setpoint values')
plot_primary.set_title('Drone steering values')
plot_primary.set_ylim(-100, 100)  # Set y-axis limit to -100 to 100

# Secondary y-axis plot (for medium values)
plot_secondary = figure.add_subplot(3, 1, 3)
plot_secondary.set_xlabel('Time')
plot_secondary.set_ylabel('PWM output')
# plot_secondary.set_title('Live Timeseries Data - Medium Values')
plot_secondary.set_ylim(900, 2100)  # Set y-axis limit to start from 900

# Tertiary y-axis plot (for large values)
plot_tertiary = figure.add_subplot(3, 1, 2)
# plot_tertiary.set_xlabel('Time')
plot_tertiary.set_ylabel('error Values')
# plot_tertiary.set_title('Live Timeseries Data - Large Values')
plot_tertiary.set_ylim(-100, 100)  # Set y-axis limit to start from 0

all_x_data = []
all_y_data_primary = []
all_y_data_secondary = []
all_y_data_tertiary = []

lines_primary = []
lines_secondary = []
lines_tertiary = []

# Define a function to initialize plot lines based on the number of data points and labels
def initialize_plot_lines(num_lines_group1, num_lines_group2, num_lines_group3, group1_labels, group2_labels, group3_labels):
    global all_y_data_primary, all_y_data_secondary, all_y_data_tertiary, rolling_y_data_primary, rolling_y_data_secondary, rolling_y_data_tertiary, lines_primary, lines_secondary, lines_tertiary
    # Initialize empty lists for y-axis data
    all_y_data_primary = [[] for _ in range(num_lines_group1)]
    all_y_data_secondary = [[] for _ in range(num_lines_group2)]
    all_y_data_tertiary = [[] for _ in range(num_lines_group3)]
    rolling_y_data_primary = [[] for _ in range(num_lines_group1)]
    rolling_y_data_secondary = [[] for _ in range(num_lines_group2)]
    rolling_y_data_tertiary = [[] for _ in range(num_lines_group3)]
    
    # Create line objects for primary plot
    lines_primary = [plot_primary.plot([], [], label=group1_labels[i])[0] for i in range(num_lines_group1)]
    
    # Create line objects for secondary plot
    lines_secondary = [plot_secondary.plot([], [], label=group2_labels[i])[0] for i in range(num_lines_group2)]
    
    # Create line objects for tertiary plot
    lines_tertiary = [plot_tertiary.plot([], [], label=group3_labels[i])[0] for i in range(num_lines_group3)]
    
    # Add legends to the plots
    plot_primary.legend(loc='upper left')
    plot_secondary.legend(loc='upper left')
    plot_tertiary.legend(loc='upper left')

# Add the plot to Tkinter
plot_canvas = FigureCanvasTkAgg(figure, root)
plot_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Start and stop buttons
start_button = tk.Button(root, text="Start", command=start_serial_connection)
start_button.pack(side=tk.LEFT)
stop_button = tk.Button(root, text="Stop", command=stop_serial_connection)
stop_button.pack(side=tk.LEFT)

# Rolling window toggle button
rolling_button = tk.Button(root, text="Enable Rolling Window", command=toggle_rolling_window)
rolling_button.pack(side=tk.LEFT)

# Actual values window button
values_window_button = tk.Button(root, text="Show Actual Values", command=create_actual_values_window)
values_window_button.pack(side=tk.LEFT)

# Create a status bar
status_bar = ttk.Label(root, text="Disconnected", relief=tk.SUNKEN, anchor=tk.W)
status_bar.pack(side=tk.RIGHT, fill=tk.X)

# Tkinter main loop
root.mainloop()
