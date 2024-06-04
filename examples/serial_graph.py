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

def read_serial_and_plot():
    global running, initialized, ser
    while running:
        try:
            if ser and ser.is_open:  # Check if serial connection is open
                # Read a line from the serial port
                data = ser.readline().decode().strip()
                #print(data)
                if data:
                    # Split the data into individual label-value pairs
                    pairs = data.split('\t')

                    # Initialize lists to store values for different groups
                    group1_labels = []
                    group1_values = []
                    group2_labels = []
                    group2_values = []

                    # Loop through each label-value pair
                    for pair in pairs:
                        try:
                            label, value = pair.split(':')
                            label = label.strip()
                            value = float(value.strip())
                        except ValueError:
                            continue  # Skip if there's an error

                        # Check the starting character of the label
                        if label[0] in ['E', 'D', 'R']:
                            group1_labels.append(label)
                            group1_values.append(value)
                        elif label[0] == 'M':
                            group2_labels.append(label)
                            group2_values.append(value)

                    # Combine labels and values for each group
                    group1_data = list(zip(group1_labels, group1_values))
                    group2_data = list(zip(group2_labels, group2_values))
                    #print("Group1", group1_data)
                    #print("Group2", group2_data)

                    if not initialized:
                        initialize_plot_lines(len(group1_data), len(group2_data), group1_labels, group2_labels)
                        initialized = True

                    # Update the plot
                    update_plot(group1_data, group2_data)

                    # Update the actual values window if active
                    if actual_values_window_active:
                        update_actual_values_window(group1_data, group2_data)
            else:
                print("Serial connection is not open")
                time.sleep(1)  # Wait for 1 second before retrying

        except Exception as e:
            print("Error processing serial data:", e)

def update_plot(group1_data, group2_data):
    global x_data

    x_data.append(len(x_data))

    # Ensure x_data length matches the rolling window size if enabled
    if rolling_window_active and len(x_data) > window_size:
        x_data = x_data[-window_size:]

    for i, (label, value) in enumerate(group1_data):
        if i < len(lines_primary):
            y_data_primary[i].append(value)
            if rolling_window_active and len(y_data_primary[i]) > window_size:
                y_data_primary[i] = y_data_primary[i][-window_size:]
            lines_primary[i].set_data(range(len(y_data_primary[i])), y_data_primary[i])

    for i, (label, value) in enumerate(group2_data):
        if i < len(lines_secondary):
            y_data_secondary[i].append(value)
            if rolling_window_active and len(y_data_secondary[i]) > window_size:
                y_data_secondary[i] = y_data_secondary[i][-window_size:]
            lines_secondary[i].set_data(range(len(y_data_secondary[i])), y_data_secondary[i])

    plot_primary.relim()
    plot_primary.autoscale_view()
    plot_secondary.relim()
    plot_secondary.autoscale_view()
    plot_canvas.draw()

# Function to start the serial connection
def start_serial_connection():
    global ser, running, initialized, x_data, y_data_primary, y_data_secondary, baud_rate, serial_port
    try:
        ser = serial.Serial(serial_port, baud_rate)  # Adjust to the correct port and baudrate
        running = True
        initialized = False
        x_data = []
        y_data_primary = []
        y_data_secondary = []
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
    global rolling_window_active
    rolling_window_active = not rolling_window_active
    if rolling_window_active:
        rolling_button.config(text="Disable Rolling Window")
    else:
        rolling_button.config(text="Enable Rolling Window")
    print(f"Rolling window {'enabled' if rolling_window_active else 'disabled'}")

# Function to create and update the actual values window
def create_actual_values_window():
    global values_labels, actual_values_window_active
    actual_values_window = tk.Toplevel(root)
    actual_values_window.title("Actual Values")
    actual_values_window.geometry("200x400")  # Set the size of the window
    actual_values_window_active = True
    values_labels = []
    for i in range(len(y_data_primary) + len(y_data_secondary)):
        label_text = f"Value {i + 1}: "
        label = ttk.Label(actual_values_window, text=label_text, anchor='e', width=30)
        label.pack(anchor=tk.W, padx=10, pady=2)
        values_labels.append(label)

def update_actual_values_window(group1_data, group2_data):
    global values_labels
    combined_data = group1_data + group2_data
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
plot_primary = figure.add_subplot(2, 1, 1)
plot_primary.set_xlabel('Time')
plot_primary.set_ylabel('Small Values (-100 to 100)')
plot_primary.set_title('Live Timeseries Data - Small Values')
plot_primary.set_ylim(-100, 100)  # Set y-axis limit to -100 to 100

# Secondary y-axis plot (for large values)
plot_secondary = figure.add_subplot(2, 1, 2)
plot_secondary.set_xlabel('Time')
plot_secondary.set_ylabel('Large Values (900 and above)')
plot_secondary.set_title('Live Timeseries Data - Large Values')
plot_secondary.set_ylim(900, 2100)  # Set y-axis limit to start from 900

x_data = []
y_data_primary = []
y_data_secondary = []

lines_primary = []
lines_secondary = []

# Define a function to initialize plot lines based on the number of data points and labels
def initialize_plot_lines(num_lines_group1, num_lines_group2, group1_labels, group2_labels):
    global y_data_primary, y_data_secondary, lines_primary, lines_secondary
    # Initialize empty lists for y-axis data
    y_data_primary = [[] for _ in range(num_lines_group1)]
    y_data_secondary = [[] for _ in range(num_lines_group2)]
    
    # Create line objects for primary plot
    lines_primary = [plot_primary.plot([], [], label=group1_labels[i])[0] for i in range(num_lines_group1)]
    
    # Create line objects for secondary plot
    lines_secondary = [plot_secondary.plot([], [], label=group2_labels[i])[0] for i in range(num_lines_group2)]
    
    # Add legends to the plots
    plot_primary.legend(loc='upper left')
    plot_secondary.legend(loc='upper left')

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
