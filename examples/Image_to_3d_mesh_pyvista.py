# -*- coding: utf-8 -*-
"""
Created on Sat Aug 10 21:06:34 2024

@author: 602037
"""
import pyvista as pv
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import cv2
import numpy as np
import trimesh
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.gridspec as gridspec
import os
import vtk

class ImageTo3DApp:
    def __init__(self, root):
        self.root = root
        self.image = None
        self.initialize_variables()
        self.create_menubar()
        self.create_statusbar()
        self.create_params_frame()
        self.create_matplotlib_canvas()        
        self.create_pyvista_canvas()
        #self.connect_mouse_events()
        self.load_default_image()
        

    def initialize_variables(self):
        """Initialize all the necessary variables for the application."""
        # Initialize azimuth and elevation
        self.azim = -90
        self.elev = 90

        # Initialize reverse height flag
        self.reverse_height = False

        # Initialize preprocessing flags
        self.hsv_adjustment_on = tk.BooleanVar(value=False)
        self.color_balance_on = tk.BooleanVar(value=False)

        # Initialize variables for sliders
        self.height_scale = tk.DoubleVar(value=0.5)
        self.smooth_scale = tk.DoubleVar(value=1.0)
        self.resolution_scale = tk.DoubleVar(value=0.3)
        self.edge_threshold = tk.DoubleVar(value=100)
        self.min_height = tk.DoubleVar(value=0.0)
        self.max_height = tk.DoubleVar(value=1.0)
        self.edge_detection_on = tk.BooleanVar(value=False)
        self.inverse_colors_on = tk.BooleanVar(value=False)
        self.reverse_height_detection_on = tk.BooleanVar(value=False)
        self.blur_amount = tk.DoubleVar(value=0.0)
        self.z_offset = tk.DoubleVar(value=0.0)

        # Initialize HSV adjustment variables
        self.hsv_saturation = tk.DoubleVar(value=0.0)
        self.hsv_value = tk.DoubleVar(value=0.0)

        # Initialize contrast and brightness variables
        self.contrast = tk.DoubleVar(value=1.0)
        self.brightness = tk.DoubleVar(value=0.0)

        # Initialize file type
        self.file_type = 'stl'  # Default file type

        self.image = None
        self.mesh = None

        self.is_dragging = False
        self.last_x, self.last_y = None, None
        
        # Initialize camera variable
        self.capture = None
        self.is_camera_open = False
        self.camera_window = None
        self.camera_index = 0        


    def create_menubar(self):
        """Create the menu bar and add menu items."""
        self.menubar = tk.Menu(self.root)
        self.root.config(menu=self.menubar)

        file_menu = tk.Menu(self.menubar, tearoff=0)
        file_menu.add_command(label="Open Image", command=self.open_image)
        file_menu.add_separator()
        file_menu.add_command(label="Save as STL", command=lambda: self.set_file_type('stl'))
        file_menu.add_command(label="Save as OBJ", command=lambda: self.set_file_type('obj'))
        file_menu.add_command(label="Save as PLY", command=lambda: self.set_file_type('ply'))
        file_menu.add_separator()
        file_menu.add_command(label="View 3D Model", command=self.view_3d_file)
        file_menu.add_separator()
        file_menu.add_command(label="Capture Image", command=self.open_camera_window)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.exit_app)
        self.menubar.add_cascade(label="File", menu=file_menu)

        info_menu = tk.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label="Info", menu=info_menu)
        info_menu.add_command(label="About", command=self.show_info)


    def create_statusbar(self):
        """Create the status bar at the bottom of the window."""
        self.statusbar = ttk.Label(self.root, text="Welcome to Image to 3D Converter", relief=tk.SUNKEN, anchor=tk.W)
        self.statusbar.pack(side=tk.BOTTOM, fill=tk.X)

    def create_params_frame(self):
        """Create a scrollable parameter frame with all the sliders and buttons."""
        # Create a canvas to hold the parameters and add a scrollbar
        self.params_canvas = tk.Canvas(self.root, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.params_canvas.yview)
        self.params_canvas.configure(yscrollcommand=self.scrollbar.set)
    
        # Pack the canvas and scrollbar into the left side of the root window
        self.scrollbar.pack(side=tk.LEFT, fill=tk.Y)
        self.params_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=False)
    
        # Create a frame inside the canvas to hold all the widgets
        self.params_frame = ttk.Frame(self.params_canvas)
        
        # Place the frame in the canvas
        self.params_canvas.create_window((0, 0), window=self.params_frame, anchor="nw")
    
        # Bind the canvas to configure scroll region whenever the size of the frame changes
        self.params_frame.bind("<Configure>", self.on_frame_configure)
    
        # Add all the widgets to params_frame as before
        self.calculate_button = ttk.Button(self.params_frame, text="Calculate Mesh", command=self.calculate_and_plot)
        self.calculate_button.pack(pady=5)
    
        self.add_slider("Resolution Scale", self.resolution_scale, 0.01, 1.0)
        self.add_slider("Contrast", self.contrast, 0.0, 3.0)
        self.add_slider("Brightness", self.brightness, -100, 100)
    
        self.Toggle_Edge_button = ttk.Button(self.params_frame, text="Toggle Edge Detect", command=self.toggle_edge_detection)
        self.Toggle_Edge_button.pack(pady=5)
    
        self.add_slider("Edge Threshold", self.edge_threshold, 0.0, 255.0)
        self.add_slider("Smoothness", self.smooth_scale, 0.0, 1.0)
        self.add_slider("Blur Amount", self.blur_amount, 0.0, 15.0)
        self.add_slider("Z Offset", self.z_offset, -1.0, 1.0)
        self.add_slider("Height Scale", self.height_scale, 0.0, 1.0)
        self.add_slider("Min Height", self.min_height, 0.0, 1.0)
        self.add_slider("Max Height", self.max_height, 0.0, 1.0)
    
        self.invert_colors_button = ttk.Button(self.params_frame, text="Invert Colors", command=self.invert_colors)
        self.invert_colors_button.pack(pady=5)
    
        self.reverse_button = ttk.Button(self.params_frame, text="Reverse Height", command=self.toggle_reverse_height_detection)
        self.reverse_button.pack(pady=5)
    
        self.add_checkbutton("HSV Adjustment", self.hsv_adjustment_on)
        self.add_checkbutton("Color Balance", self.color_balance_on)
    
        self.front_view_button = ttk.Button(self.params_frame, text="Front View", command=self.set_front_view)
        self.front_view_button.pack(pady=5)
    
        self.top_view_button = ttk.Button(self.params_frame, text="Top View", command=self.set_top_view)
        self.top_view_button.pack(pady=5)
    
    def on_frame_configure(self, event):
        """Reset the scroll region to encompass the inner frame."""
        self.params_canvas.configure(scrollregion=self.params_canvas.bbox("all"))


         
    def show_info(self):
        info_message = (
            "2D Image to 3d mesh viewer\n\n"
            "This application allows you to:\n"
        )
        messagebox.showinfo("About This Application", info_message)
        
    def create_pyvista_canvas(self):
        """Create a Tkinter canvas for displaying PyVista plots."""
        # Initialize PyVista plotter
        self.plotter = pv.Plotter(off_screen=True)
        
        # Create a Tkinter canvas to display PyVista images
        self.pv_canvas = tk.Canvas(self.root)  # Use a unique name for the PyVista canvas
        self.pv_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Bind resize event to update image when the canvas size changes
        self.pv_canvas.bind("<Configure>", self.on_resize)
      
        # Embed the PyVista plotter in Tkinter Canvas
        self.plotter_widget = self.pv_canvas
        self.plotter_widget.bind("<Button-1>", self.on_mouse_click)
        self.plotter_widget.bind("<B1-Motion>", self.on_mouse_drag)
        self.plotter_widget.bind("<MouseWheel>", self.on_mouse_scroll)
    
    
    def on_resize(self, event):
        # Detect the new size of the canvas
        canvas_width = event.width
        canvas_height = event.height
        
        # Adjust the size of the PyVista plotter
        self.plotter.window_size = (canvas_width, canvas_height)
        
        # Re-render the image to fit the new canvas size
        self.render_pyvista()
    
    def create_matplotlib_canvas(self):
        """Create the Matplotlib canvas for displaying images and 3D models."""
        self.fig = plt.figure(figsize=(6, 12))  # Larger figure size to make 3D view bigger
        gs = gridspec.GridSpec(3, 1, figure=self.fig)
    
        self.ax_img = self.fig.add_subplot(gs[0, 0])  # Original Image in row 1, column 1
        self.ax_gray = self.fig.add_subplot(gs[1, 0])  # Grayscale Image in row 2, column 1
        self.ax_edges = self.fig.add_subplot(gs[2, 0])  # Edge Image in row 3, column 1
    
        self.ax_img.axis('off')
        self.ax_gray.axis('off')
        self.ax_edges.axis('off')
    
        self.fig.tight_layout()
        
        # Use a unique name for the Matplotlib canvas
        self.mpl_canvas = FigureCanvasTkAgg(self.fig) #, master=self.root
        self.mpl_canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=False)
        self.mpl_canvas.draw()

    def add_slider(self, label, variable, from_, to_):
        """Helper function to add a labeled slider to the params frame."""
        ttk.Label(self.params_frame, text=label).pack(pady=5)
        slider = ttk.Scale(self.params_frame, from_=from_, to=to_, orient=tk.HORIZONTAL, variable=variable, command=self.update_mesh)
        slider.pack(fill=tk.X)

    def add_button(self, label, command):
        """Helper function to add a button to the params frame."""
        button = ttk.Button(self.params_frame, text=label, command=command)
        button.pack(pady=5)

    def add_checkbutton(self, label, variable):
        """Helper function to add a checkbutton to the params frame."""
        checkbutton = ttk.Checkbutton(self.params_frame, text=label, variable=variable, command=self.update_mesh)
        checkbutton.pack(pady=5)

    def add_nav_button(self, label, command):
        """Helper function to add a navigation button to the nav frame."""
        button = ttk.Button(self.nav_frame, text=label, command=command)
        button.pack(side=tk.RIGHT, padx=5)

    def load_default_image(self):
        """Load the default image from the specified path."""
        default_image_path = 'C:/temp/images/cat.jpg'
        try:
            self.image = cv2.imread(default_image_path)
            if self.image is None:
                raise FileNotFoundError(f"Default image not found at: {default_image_path}")
            self.statusbar.config(text=f"Loaded default image: {default_image_path}")
            self.update_mesh()  # Optionally, update mesh using the default image
        except Exception as e:
            self.statusbar.config(text=f"Error loading default image: {str(e)}")
            
    def open_image(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.jpg *.jpeg *.png")])
        if file_path:
            self.image = cv2.imread(file_path)
            if self.image is None:
                self.statusbar.config(text="Error loading image")
            else:
                self.statusbar.config(text=f"Loaded image: {file_path}")
                self.update_mesh()

    def open_camera_window(self):
        """Open a new window with the camera feed and capture button."""
        self.camera_window = tk.Toplevel(self.root)
        self.camera_window.title("Camera Viewer")

        # Create a dropdown menu to select the camera
        camera_label = tk.Label(self.camera_window, text="Select Camera:")
        camera_label.pack(pady=5)

        # Get available cameras
        camera_indices = self.detect_cameras()

        # Create a combobox to select camera index
        self.camera_combobox = ttk.Combobox(self.camera_window, values=camera_indices)
        self.camera_combobox.set(str(self.camera_index))  # Set default index
        self.camera_combobox.pack(pady=5)

        # Create a button to start the selected camera
        start_button = tk.Button(self.camera_window, text="Start Camera", command=self.start_camera)
        start_button.pack(pady=5)

        # Create a label for the video feed
        self.video_label = tk.Label(self.camera_window)
        self.video_label.pack(pady=5)

        # Create a button to capture the image
        capture_button = tk.Button(self.camera_window, text="Capture Image", command=self.capture_image)
        capture_button.pack(pady=20)

        # Handle window close event
        self.camera_window.protocol("WM_DELETE_WINDOW", self.on_camera_window_close)
        
    def detect_cameras(self):
        """Detect available cameras and return a list of indices."""
        indices = []
        
        for i in range(2):  # Check first 10 indices
            capture = cv2.VideoCapture(i,cv2.CAP_DSHOW)
            if capture.isOpened():
                indices.append(i)
                capture.release()
               
        return indices
    
    def start_camera(self):
        """Start the camera feed based on selected camera index."""
        if self.is_camera_open:
            self.stop_camera()

        # Get selected camera index from the combobox
        try:
            self.camera_index = int(self.camera_combobox.get())
        except ValueError:
            self.camera_index = 0  # Default to 0 if invalid input

        # Open the selected camera
        self.capture = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.capture.isOpened():
            print("Error: Camera not accessible.")
            return

        self.is_camera_open = True
        self.update_frame()

    def update_frame(self):
        """Update the video feed frame by frame."""
        if self.is_camera_open and self.camera_window and self.video_label:
            ret, frame = self.capture.read()
            if ret:
                # Convert the image to RGB (OpenCV uses BGR by default)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Convert to PIL Image and then to ImageTk
                img = Image.fromarray(frame_rgb)
                img_tk = ImageTk.PhotoImage(image=img)

                try:
                    # Update the label with the new image
                    self.video_label.config(image=img_tk)
                    self.video_label.image = img_tk  # Keep a reference to avoid garbage collection
                except tk.TclError:
                    # Handle case where the widget no longer exists
                    print("Error: The camera window or label no longer exists.")
                    self.stop_camera()
                    return

            # Call this method again after a short delay
            self.root.after(10, self.update_frame)

    def capture_image(self):
        """Capture the current frame and save it."""
        if self.is_camera_open:
            ret, frame = self.capture.read()
            if ret:
                # Convert the image to RGB (OpenCV uses BGR by default)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Convert to PIL Image
                img = Image.fromarray(frame_rgb)
                
                # Save the image
                save_path = filedialog.asksaveasfilename(defaultextension=".jpg", filetypes=[("JPEG files", "*.jpg"), ("PNG files", "*.png")])
                if save_path:
                    img.save(save_path)
            else:
                print("Error: Failed to capture image.")

    def on_camera_window_close(self):
        """Handle the closing of the camera window."""
        self.stop_camera()
        if self.camera_window:
            self.camera_window.destroy()

    def stop_camera(self):
        """Release the camera and stop the feed."""
        if self.is_camera_open:
            self.is_camera_open = False
            if self.capture:
                self.capture.release()
    '''                
        def capture_image(self):
            # Open a dialog to select a camera index
            camera_index = simpledialog.askinteger("Select Camera", "Enter camera index:", initialvalue=0, minvalue=0)
            
            if camera_index is not None:
                # Try to open the camera
                self.capture = cv2.VideoCapture(camera_index)
                
                if not self.capture.isOpened():
                    print("Error: Camera not accessible.")
                    return
    
                # Capture an image
                ret, frame = self.capture.read()
                if ret:
                    # Convert the image to RGB (OpenCV uses BGR by default)
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # Convert to PIL Image and then to ImageTk
                    img = Image.fromarray(frame_rgb)
                    img_tk = ImageTk.PhotoImage(image=img)
    
                    # Display the image in a Tkinter window
                    image_window = tk.Toplevel(self.root)
                    image_label = tk.Label(image_window, image=img_tk)
                    image_label.image = img_tk  # Keep a reference to avoid garbage collection
                    image_label.pack()
                    
                    # Optionally, save the image
                    save_path = filedialog.asksaveasfilename(defaultextension=".jpg", filetypes=[("JPEG files", "*.jpg"), ("PNG files", "*.png")])
                    if save_path:
                        img.save(save_path)
                else:
                    print("Error: Failed to capture image.")
                
                # Release the camera
                self.capture.release()
     '''           
    def view_3d_file(self):
        file_path = filedialog.askopenfilename(filetypes=[
            ("All files", "*.*"),
            ("STL files", "*.stl"),
            ("OBJ files", "*.obj"),
            ("PLY files", "*.ply")
        ])
        if file_path:
            try:
                # Create a PyVista plotter object
                plotter = pv.Plotter()
    
                # Load the mesh file with PyVista
                mesh = pv.read(file_path)
    
                # Debugging output to check point_data keys
                print("Mesh point data keys:", mesh.point_data.keys())
                
                # Check if the mesh has color information
                if 'RGB' in mesh.point_data:
                    print("Mesh contains color information.")
                    print("Color data shape:", mesh.point_data['RGB'].shape)
                    print("First few colors:", mesh.point_data['RGB'][:5])
    
                    # Plot with the color information in the mesh
                    plotter.add_mesh(mesh, scalars='RGB', rgb=True, show_edges=False, opacity=1)
                else:
                    print("No color information found. Using default color.")
                    plotter.add_mesh(mesh, color='lightgreen', show_edges=True, opacity=1)
    
                # Set the title to the filename
                plotter.add_title(os.path.basename(file_path))
    
                # Show grid
                plotter.show_grid()  
    
                # Show the plot
                plotter.show()
    
                # Update the status bar with the file path
                self.statusbar.config(text=f"Viewing 3D file: {file_path}")
            except Exception as e:
                self.statusbar.config(text=f"Error viewing 3D file: {str(e)}")
        else:
            self.statusbar.config(text="File selection canceled.")


    def set_file_type(self, file_type):
            self.file_type = file_type
            #print(f"Selected file type: {self.file_type}")
            self.save_3d_model()


    def save_3d_model(self):
        if self.mesh is not None:
            # Determine the appropriate file type and extension
            filetype_map = {
                "stl": ("STL files", "*.stl"),
                "obj": ("OBJ files", "*.obj"),
                "ply": ("PLY files", "*.ply")
            }
    
            if self.file_type not in filetype_map:
                self.statusbar.config(text="Unsupported file type.")
                return
    
            filetype_description, filetype_extension = filetype_map[self.file_type]
    
            # Display save dialog with filtered filetypes
            file_path = filedialog.asksaveasfilename(
                defaultextension=f".{self.file_type}",
                filetypes=[(filetype_description, filetype_extension)],
                title="Save 3D Model",
                initialfile=f"model.{self.file_type}"  # Set a default filename with the correct extension
            )
    
            print(f"Selected file path: {file_path}")  # Debugging line
    
            if file_path:
                try:
                    vtk_mesh = self.mesh  # Already in PolyData format
    
                    # Check for color information in the mesh
                    if 'RGB' in self.mesh.point_data:
                        print("Mesh contains color information.")
                        print(f"Color data shape: {self.mesh.point_data['RGB'].shape}")
                        print(f"First few colors: {self.mesh.point_data['RGB'][:5]}")
    
                        # Ensure color data is in the correct format (0-255)
                        colors = self.mesh.point_data['RGB']
    
                        # If colors are in [0, 1], scale to [0, 255]
                        if colors.max() <= 1.0:
                            colors = (colors * 255).astype(np.uint8)
                        else:
                            colors = colors.astype(np.uint8)
    
                        vtk_colors = numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
                        vtk_colors.SetName('RGB')
                        vtk_mesh.GetPointData().SetScalars(vtk_colors)
                    else:
                        print("Mesh does not contain color information.")
                        # Assign default white color if no color info is available
                        num_points = vtk_mesh.GetNumberOfPoints()
                        white_colors = numpy_to_vtk(np.full((num_points, 3), 255, dtype=np.uint8), deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
                        white_colors.SetName('RGB')
                        vtk_mesh.GetPointData().SetScalars(white_colors)
    
                    if self.file_type == "ply":
                        writer = vtk.vtkPLYWriter()
                        writer.SetFileName(file_path)
                        writer.SetInputData(vtk_mesh)
                        writer.SetFileTypeToASCII()
                        writer.SetArrayName('RGB')
                        writer.Write()
                    elif self.file_type == "obj":
                        renderer = vtk.vtkRenderer()
                        render_window = vtk.vtkRenderWindow()
                        render_window.AddRenderer(renderer)
                        render_window_interactor = vtk.vtkRenderWindowInteractor()
                        render_window_interactor.SetRenderWindow(render_window)
    
                        mapper = vtk.vtkPolyDataMapper()
                        mapper.SetInputData(vtk_mesh)
                        actor = vtk.vtkActor()
                        actor.SetMapper(mapper)
                        renderer.AddActor(actor)
    
                        render_window.Render()
    
                        exporter = vtk.vtkOBJExporter()
                        exporter.SetFilePrefix(os.path.splitext(file_path)[0])
                        exporter.SetInput(render_window)
                        exporter.Write()
                    else:
                        self.mesh.save(file_path)
                    
                    self.statusbar.config(text=f"Saved 3D model to: {file_path}")
                except Exception as e:
                    self.statusbar.config(text=f"Error saving 3D model: {str(e)}")
                    print(f"Exception occurred: {e}")  # Debugging line
            else:
                self.statusbar.config(text="Save operation canceled.")
        else:
            self.statusbar.config(text="No 3D model to save.")


    def calculate_and_plot(self):
        if self.image is not None:
            self.update_mesh()
        else:
            self.statusbar.config(text="No image to process")

    def update_mesh(self, event=None):
        if self.image is None:
            return
    
        self.clear_previous_plots()
    
        # Retrieve UI values/settings
        settings = self.get_ui_settings()

        # Get the dimensions of the image (height, width, number of channels)
        height, width = self.image.shape[:2]
        
        # Calculate the aspect ratio
        self.aspect_ratio = width / height
        
        # Print the aspect ratio
        #print(f"Aspect Ratio: {self.aspect_ratio:.2f}")
        
        # Resize the image and display it in the 2D plot
        image_resized = self.resize_image(self.image, settings['resolution_scale'])
        self.display_resized_image(image_resized)
    
        # Convert the resized image to grayscale
        gray_image = self.preprocess_image(image_resized, settings)
    
        # Display the grayscaled and preprocessed image
        self.ax_gray.imshow(gray_image, cmap='gray')
        self.ax_gray.axis('on')
    
        # Perform edge detection if enabled and get the height map
        height_map = self.get_height_map(gray_image, settings)
    
    
        # Adjust height map and check for errors
        height_map = self.adjust_height_map(height_map, settings)
        if height_map is None:
            return
    
        # Downscale the height map
        height_map = self.downscale_height_map(height_map)
    
        # Ensure the height map has valid dimensions
        #if not self.is_valid_height_map(height_map):
        #    return
    
        # Generate vertices and colors for the mesh
        vertices, colors = self.generate_mesh_data(height_map, image_resized, settings)
    
        # Generate faces for the mesh
        faces = self.generate_mesh_faces(height_map)
    
        # Create and update the PyVista mesh
        self.update_pyvista_mesh(vertices, faces, colors)
    
        # Redraw the canvas to update the plots
        self.fig.canvas.draw()
        
        # Update the Tkinter canvas with the PyVista image
        #self.update_tkinter_canvas()
    
        self.render_pyvista()
    
    def clear_previous_plots(self):
        self.ax_img.clear()
        self.ax_gray.clear()
        self.ax_edges.clear()
    
    def get_ui_settings(self):
        return {
            'height_scale': self.height_scale.get(),
            'smoothness': self.smooth_scale.get(),
            'resolution_scale': self.resolution_scale.get(),
            'edge_threshold': self.edge_threshold.get(),
            'min_height': self.min_height.get(),
            'max_height': self.max_height.get(),
            'edge_detection_on': self.edge_detection_on.get(),
            'reverse_height_detection_on': self.reverse_height_detection_on.get(),
            'hsv_adjustment_on': self.hsv_adjustment_on.get(),
            'color_balance_on': self.color_balance_on.get(),
            'contrast': self.contrast.get(),
            'brightness': self.brightness.get(),
            'blur_amount': self.blur_amount.get(),
            'z_offset': self.z_offset.get(),
        }
    
    def resize_image(self, image, resolution_scale):
        new_size = (int(image.shape[1] * resolution_scale), int(image.shape[0] * resolution_scale))
        return cv2.resize(image, new_size)
    
    def display_resized_image(self, image_resized):
        self.ax_img.imshow(cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB))
        self.ax_img.get_xaxis().set_visible(False)
    
    def preprocess_image(self, image_resized, settings):
        gray_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.convertScaleAbs(gray_image, alpha=settings['contrast'], beta=settings['brightness'])
        
        if settings['blur_amount'] > 0:
            gray_image = cv2.GaussianBlur(gray_image, (5, 5), settings['blur_amount'])
        
        if settings['hsv_adjustment_on']:
            gray_image = self.apply_hsv_adjustment(gray_image)
        
        if settings['color_balance_on']:
            gray_image = self.apply_color_balance(gray_image)
        
        return gray_image
    
    def get_height_map(self, gray_image, settings):
        if settings['edge_detection_on']:
            edges = cv2.Canny(gray_image, threshold1=settings['edge_threshold'], threshold2=settings['edge_threshold'] * 2)
            if settings['smoothness'] > 0:
                edges = cv2.GaussianBlur(edges, (5, 5), settings['smoothness'] * 10)
            self.ax_edges.imshow(edges, cmap='gray')
            self.ax_edges.set_title('Edges')
            self.ax_edges.axis('off')
            return edges.astype(float) / 255.0
        else:
            self.ax_edges.axis('off')
            return gray_image.astype(float) / 255.0
    
    def adjust_height_map(self, height_map, settings):
        if settings['reverse_height_detection_on']:
            height_map = (1 - height_map) * settings['height_scale']
        else:
            height_map = height_map * settings['height_scale']
    
        if height_map is None or height_map.size == 0:
            print("Error: height_map is None or empty.")
            return None
    
        return height_map
    
    def downscale_height_map(self, height_map):
        return height_map[::2, ::2]
    
    def is_valid_height_map(self, height_map):
        if height_map.shape[0] == 0 or height_map.shape[1] == 0:
            print("Error: height_map has invalid shape.")
            return False
        return True
    
    def generate_mesh_data(self, height_map, image_resized, settings):
                
        h, w = height_map.shape
        x = np.linspace(0, self.aspect_ratio, w)
        y = np.linspace(0, 1, h)
        x, y = np.meshgrid(x, y)
        y = 1 - y  # Mirror the y-axis
    
        vertices = np.column_stack((x.ravel(), y.ravel(), height_map.ravel()))
        vertices[:, 2] += settings['z_offset']
        vertices[:, 2] = np.clip(vertices[:, 2], settings['min_height'], settings['max_height'])
    
        colors = image_resized[::2, ::2].reshape(-1, 3) / 255.0
        return vertices, colors
    
    def generate_mesh_faces(self, height_map):
        h, w = height_map.shape
        faces = []
        for i in range(h - 1):
            for j in range(w - 1):
                idx = i * w + j
                faces.append([3, idx, idx + 1, idx + w])
                faces.append([3, idx + 1, idx + w + 1, idx + w])
        return np.concatenate(faces)
    
    def update_pyvista_mesh(self, vertices, faces, colors):
        self.mesh = pv.PolyData(vertices)
        self.mesh.faces = faces
        self.mesh.point_data['Colors'] = colors
        # Attach colors if they exist
        # Attach colors if they exist
        if colors is not None and len(colors) > 0:
            self.mesh.point_data['RGB'] = (colors * 255).astype(np.uint8)

        self.mesh.points[:, 2] = np.clip(self.mesh.points[:, 2], self.min_height.get(), self.max_height.get())
        
        self.mesh=self.mesh               
        self.plotter.clear()
                
        self.plotter.add_mesh(self.mesh, scalars='Colors', rgb=True, show_edges=True)
        
        # Add 3D axes
        self.plotter.show_axes()  # You can customize the axes here if needed
        
        # Show grid
        self.plotter.show_grid()  # This adds a grid to the 3D plot
        #self.plotter.show_bounds(location='all')
        
        # Add color legend for height
        # self.plotter.add_scalar_bar(title="Height", vertical=False, title_font_size=12, label_font_size=10)
   
        # Optional: Explicitly add axes if more customization is required
        #self.plotter.add_axes(interactive=True)  # Adds interactive axes
        self.plotter.view_isometric()
    
        self.rotate_camera_180_degrees()
    
    def rotate_camera_180_degrees(self):
        camera = self.plotter.camera
        current_position = camera.position
        current_focal_point = camera.focal_point
        camera.position = (current_position[0], -current_position[1], current_position[2])
        camera.focal_point = (current_focal_point[0], -current_focal_point[1], current_focal_point[2])
        self.plotter.camera = camera
        self.plotter.reset_camera()
        self.plotter.render()
    
    def render_pyvista(self):
        self.plotter.render()
        img = self.plotter.screenshot() #window_size= (1000, 1000)
        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img)
        self.pv_canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
        self.pv_canvas.image = img_tk
        self.pv_canvas.update_idletasks()
        
    def solidify_body(self):
        """solidify body     """        
        if self.mesh is not None:
            try:
                # Create a copy of the mesh to modify
                top_mesh = self.mesh.copy()
                
                # Set the top mesh at a certain height (e.g., z=height_scale)
                height_scale = self.height_scale.get()
                top_mesh.vertices[:, 2] = top_mesh.vertices[:, 2] + height_scale
                
                # Get bounds for creating the bottom plane
                min_x, max_x = top_mesh.vertices[:, 0].min(), top_mesh.vertices[:, 0].max()
                min_y, max_y = top_mesh.vertices[:, 1].min(), top_mesh.vertices[:, 1].max()
                
                # Create the bottom plane at z=0
                plane_vertices = np.array([
                    [min_x, min_y, 0],
                    [max_x, min_y, 0],
                    [max_x, max_y, 0],
                    [min_x, max_y, 0]
                ])
                plane_faces = np.array([
                    [0, 1, 2],
                    [0, 2, 3]
                ])
                plane_mesh = trimesh.Trimesh(vertices=plane_vertices, faces=plane_faces, process=False)
                
                # Create side faces connecting top mesh to bottom plane
                top_vertices = top_mesh.vertices
                bottom_vertices = np.copy(top_vertices)
                bottom_vertices[:, 2] = 0
                
                # Create side faces
                side_faces = []
                for i in range(len(top_mesh.faces)):
                    face = top_mesh.faces[i]
                    for j in range(len(face)):
                        v1 = face[j]
                        v2 = face[(j + 1) % len(face)]
                        v3 = len(bottom_vertices) + v1
                        v4 = len(bottom_vertices) + v2
                        side_faces.append([v1, v2, v3])
                        side_faces.append([v2, v4, v3])
                
                # Combine the meshes
                combined_vertices = np.vstack((bottom_vertices, top_vertices))
                combined_faces = np.vstack((plane_faces, top_mesh.faces + len(bottom_vertices), side_faces))
                
                # Create the combined mesh
                solidified_body = trimesh.Trimesh(vertices=combined_vertices, faces=combined_faces, process=False)
                
                # Update the plot with the solidified mesh
                self.mesh = solidified_body
                
                self.statusbar.config(text="Solidified body created and updated.")
            except Exception as e:
                self.statusbar.config(text=f"Error creating solidified body: {str(e)}")
        else:
            self.statusbar.config(text="No mesh to solidify.")

    def apply_hsv_adjustment(self, image):
        # Convert grayscale image to BGR if necessary
        if len(image.shape) == 2:  # Check if the image is grayscale
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        # Convert BGR image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Apply adjustments
        if self.hsv_saturation.get() != 0:
            hsv[..., 1] = np.clip(hsv[..., 1] + self.hsv_saturation.get(), 0, 255)
        
        if self.hsv_value.get() != 0:
            hsv[..., 2] = np.clip(hsv[..., 2] + self.hsv_value.get(), 0, 255)
        
        # Convert back to BGR
        adjusted_image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
        # Convert back to grayscale
        return cv2.cvtColor(adjusted_image, cv2.COLOR_BGR2GRAY)

    def apply_color_balance(self, image):
        # Convert grayscale image to BGR
        if len(image.shape) == 2:  # Check if the image is grayscale
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        # Convert BGR image to LAB
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
        l, a, b = cv2.split(lab)
        
        # Apply histogram equalization to L channel
        l = cv2.equalizeHist(l)
        
        # Merge channels back
        lab = cv2.merge((l, a, b))
        
        # Convert LAB back to BGR
        result = cv2.cvtColor(lab, cv2.COLOR_Lab2BGR)
        
        # Convert back to grayscale
        return cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    def invert_colors(self):
        if self.image is not None:
            # Invert colors if the button state is not currently set to inverted
            if not self.inverse_colors_on.get():
                self.image = cv2.bitwise_not(self.image)
                self.inverse_colors_on.set(True)  # Update state to indicate colors are inverted
            else:
                # Reset colors if the button state is currently set to inverted
                self.image = cv2.bitwise_not(self.image)  # Invert again to reset to original
                self.inverse_colors_on.set(False)  # Update state to indicate colors are reset
    
            # Update button text based on the new state
            if self.inverse_colors_on.get():
                self.invert_colors_button.config(text="Reset Colors")
            else:
                self.invert_colors_button.config(text="Invert Colors")
            
            self.update_mesh()  # Update mesh to reflect color changes
 

    def toggle_edge_detection(self):
        current_state = self.edge_detection_on.get()
        self.edge_detection_on.set(not current_state)  # Toggle the state
        
        # Update button text based on the new state
        if self.edge_detection_on.get():
            self.Toggle_Edge_button.config(text="Reset Edge detect")
        else:
            self.Toggle_Edge_button.config(text="Toggle Edge detect")
        
        self.update_mesh()  # Update mesh to reflect changes
        
    def toggle_reverse_height_detection(self):
        current_state = self.reverse_height_detection_on.get()
        self.reverse_height_detection_on.set(not current_state)  # Toggle the state

        # Update button text based on the new state
        if self.reverse_height_detection_on.get():
            self.reverse_button.config(text="Reset Height")
        else:
            self.reverse_button.config(text="Reverse Height")
            
        self.update_mesh()  # Update mesh to reflect changes     

    def solidify_mesh(self):
        if self.mesh is not None:
            try:
                # Create a copy of the mesh to modify
                solidified_mesh = self.mesh.copy()
    
                # Adjust the Z-coordinate of the vertices to create a solidified effect
                solidified_mesh.vertices[:, 2] += 0.5  # Adjust the solidify amount as needed
                
                # Recreate the mesh with the updated vertices
                # Note: Omit vertex_colors if they do not exist
                self.mesh = trimesh.Trimesh(vertices=solidified_mesh.vertices, faces=self.mesh.faces)
                #print(dir(self.mesh))

                # Update the plot with the solidified mesh
                #self.update_mesh()
                
                self.statusbar.config(text="Mesh solidified and updated in the plot.")
            except Exception as e:
                self.statusbar.config(text=f"Error applying solidify effect: {str(e)}")
        else:
            self.statusbar.config(text="No mesh to solidify.")

    def on_mouse_click(self, event):
        # Initialize the last_x and last_y coordinates when mouse is clicked
        self.last_x = event.x
        self.last_y = event.y

    def on_mouse_drag(self, event):
        # Ensure last_x and last_y are not None
        if self.last_x is None or self.last_y is None:
            return
        
        dx = event.x - self.last_x
        dy = event.y - self.last_y
        self.last_x = event.x
        self.last_y = event.y
        
        # Adjust the camera position based on mouse movement
        camera = self.plotter.camera
        camera.position = (camera.position[0] + dx * 0.01, camera.position[1] - dy * 0.01, camera.position[2])
        self.plotter.camera = camera
        self.plotter.reset_camera()
        self.render_pyvista()

    def on_mouse_scroll(self, event):
        #print(f"Mouse scroll event delta: {event.delta}")
    
        # Determine zoom direction
        zoom_factor = 1.1 if event.delta > 0 else 1 / 1.1
    
        # Print the zoom factor for debugging
        #print(f"Applying zoom factor: {zoom_factor}")
    
        # Apply zoom
        self.plotter.camera.zoom(zoom_factor)
    
        # Ensure the plotter is updated with the new camera settings
        self.render_pyvista()
        
    def set_front_view(self):
        """Set the camera to the front view."""
        if self.plotter:
            self.plotter.view_vector((1, 0, 0))   # Front view along the xz-plane
            #self.plotter.view_isometric()
            self.render_pyvista()
    
    def set_top_view(self):
        """Set the camera to the top view."""
        if self.plotter:
            self.plotter.view_vector((0, -0.1, 1))  # Top view along the xy-plane
            #self.plotter.view_isometric()
            self.render_pyvista()

    def exit_app(self):
        """Release resources and close the application."""
        if self.capture and self.capture.isOpened():
            self.capture.release()
        self.stop_camera()
        self.root.quit()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(1700, 1000)  # Minimum size: 400x300 pixels
    app = ImageTo3DApp(root)
    # Set up the application to exit properly
    root.protocol("WM_DELETE_WINDOW", app.exit_app)
    root.mainloop()