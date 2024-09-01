import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

class DemoApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Tkinter Grid Layout with Scrollbar Demo")
        
        # Create the menu bar
        self.create_menubar()
        
        # Create the main content frame with 3 columns
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Configure grid layout for main frame
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(0, weight=1)
        
        self.main_frame.grid_columnconfigure(0, weight=0)  # Fixed width for first column
        self.main_frame.grid_columnconfigure(1, minsize=75)  # Set minimum size for second column
        self.main_frame.grid_columnconfigure(2, weight=1)
        
        # Create a frame with a scrollbar for the first column
        self.first_column_frame = ttk.Frame(self.main_frame)
        self.first_column_frame.grid(row=0, column=0, sticky="nsew")
        
        # Add a canvas to the first column frame
        self.first_column_canvas = tk.Canvas(self.first_column_frame, width=75)  # Fixed width for the first column
        self.first_column_canvas.grid(row=0, column=1, sticky="nsew")
        
        # Add a scrollbar to the left side of the canvas
        self.scrollbar = ttk.Scrollbar(self.first_column_frame, orient="vertical", command=self.first_column_canvas.yview)
        self.scrollbar.grid(row=0, column=0, sticky="ns")
        self.first_column_canvas.configure(yscrollcommand=self.scrollbar.set)
        
        # Configure the first column frame grid to expand vertically
        self.first_column_frame.grid_rowconfigure(0, weight=1)
        self.first_column_frame.grid_columnconfigure(1, weight=1)
        
        # Create a frame inside the canvas to hold the buttons
        self.buttons_frame = ttk.Frame(self.first_column_canvas)
        
        # Add the buttons frame to the canvas
        self.first_column_canvas.create_window((0, 0), window=self.buttons_frame, anchor="nw")
        
        # Add buttons to the buttons frame
        for i in range(20):  # Add more buttons to demonstrate scrolling
            btn = ttk.Button(self.buttons_frame, text=f"Button {i+1}", 
                             command=lambda i=i: self.on_button_click(i+1))
            btn.pack(pady=5, fill=tk.X)            
        
        # Configure the scroll region
        self.buttons_frame.bind("<Configure>", self.on_frame_configure)
        
        # Add LabelFrame to the second column to create a box around it
        self.second_column_frame = ttk.LabelFrame(self.main_frame, text="Column 2", padding="10")
        self.second_column_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.second_column_frame.grid_propagate(False)  # Prevent the frame from resizing based on content
        
        # Call the method to add image and text to the second column
        self.add_image_and_text_to_second_column(r"/home/rpi5/Afbeeldingen/beard_man", "Sample Text")
        
        # Add LabelFrame to the third column to create a box around it
        self.third_column_frame = ttk.LabelFrame(self.main_frame, text="Column 3", padding="10")
        self.third_column_frame.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        
        # Add content to the third column frame
        lbl2 = ttk.Label(self.third_column_frame, text="Column 3: More content")
        lbl2.pack(pady=5, fill=None, expand=False)
        
        # Create the status bar at the bottom
        self.statusbar = ttk.Label(self.root, text="Status: Ready", relief=tk.SUNKEN, anchor=tk.W)
        self.statusbar.grid(row=1, column=0, sticky="ew")
        
    def create_menubar(self):
        """Create the menu bar and add menu items."""
        self.menubar = tk.Menu(self.root)
        self.root.config(menu=self.menubar)

        file_menu = tk.Menu(self.menubar, tearoff=0)
        file_menu.add_command(label="New", command=self.menu_action_new)
        file_menu.add_command(label="Open", command=self.menu_action_open)
        file_menu.add_command(label="Save", command=self.menu_action_save)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.on_exit)  # Use self.on_exit to exit
        self.menubar.add_cascade(label="File", menu=file_menu)

        help_menu = tk.Menu(self.menubar, tearoff=0)
        help_menu.add_command(label="About", command=self.show_about_message)  # Updated to show a message box
        self.menubar.add_cascade(label="Help", menu=help_menu)

    def on_frame_configure(self, event):
        """Update the scroll region of the canvas."""
        self.first_column_canvas.configure(scrollregion=self.first_column_canvas.bbox("all"))
        
    def menu_action_new(self):
        """Dummy action for menu commands."""
        self.statusbar.config(text="Menu item new clicked")

    def menu_action_open(self):
        """Dummy action for menu commands."""
        self.statusbar.config(text="Menu item open clicked")

    def menu_action_save(self):
        """Dummy action for menu commands."""
        self.statusbar.config(text="Menu item save clicked")        

    def show_about_message(self):
        self.statusbar.config(text="Menu item about clicked")
        """Show an 'About' message box."""
        messagebox.showinfo("About", "This is a Tkinter Grid Layout with Scrollbar Demo.") 
        
    def on_button_click(self, button_number):
        """Handle button click and update status bar."""
        self.statusbar.config(text=f"Button {button_number} clicked")        
        
    def add_image_and_text_to_second_column(self, image_path, text):
        """Add an image and text to the second column frame while keeping the aspect ratio."""
        try:
            # Open the image using PIL
            image = Image.open(image_path)
            
            # Get the original dimensions of the image
            original_width, original_height = image.size
            
            # Set the maximum size for the image
            max_size = (200, 200)
            
            # Calculate the ratio and new dimensions
            ratio = min(max_size[0] / original_width, max_size[1] / original_height)
            new_width = int(original_width * ratio)
            new_height = int(original_height * ratio)
            
            # Resize the image while maintaining the aspect ratio
            image = image.resize((new_width, new_height), Image.Resampling.LANCZOS)
            
            # Convert image to PhotoImage for Tkinter
            photo = ImageTk.PhotoImage(image)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load image: {e}")
            return
        
        # Create a label to display the image
        image_label = ttk.Label(self.second_column_frame, image=photo)
        image_label.image = photo  # Keep a reference to the image
        image_label.pack(pady=5)
        
        # Create a label to display the text
        text_label = ttk.Label(self.second_column_frame, text=text)
        text_label.pack(pady=5)
            

    def on_exit(self):
        """Properly handle exiting the application."""
        self.root.destroy()

# Create the root window
root = tk.Tk()
root.minsize(900, 500)  # Minimum size: 400x300 pixels
# Create an instance of the demo application
app = DemoApp(root)

# Start the Tkinter event loop
root.mainloop()