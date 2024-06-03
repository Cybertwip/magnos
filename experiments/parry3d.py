import numpy as np
import vtk
import pyautogui

# Get screen dimensions using pyautogui
screen_width, screen_height = pyautogui.size()

# Define simulation parameters
grid_size = 250
activation_radius = 10
num_frames = 100
initial_particles = 2000

# Initialize particle states (0: inactive, 1: active)
particles = np.zeros((grid_size, grid_size, grid_size), dtype=int)

# Define central activation point
center = (grid_size // 2, grid_size // 2, grid_size // 2)

# Randomly activate a set of initial particles around the center
x = np.random.normal(center[0], activation_radius, initial_particles).astype(int)
y = np.random.normal(center[1], activation_radius, initial_particles).astype(int)
z = np.random.normal(center[2], activation_radius, initial_particles).astype(int)
particles[x, y, z] = 1

# VTK setup
points = vtk.vtkPoints()
vertices = vtk.vtkCellArray()
colors = vtk.vtkUnsignedCharArray()
colors.SetNumberOfComponents(3)
colors.SetName("Colors")

# Create a polydata object
polydata = vtk.vtkPolyData()
polydata.SetPoints(points)
polydata.SetVerts(vertices)
polydata.GetPointData().SetScalars(colors)

# Mapper
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputData(polydata)

# Actor
actor = vtk.vtkActor()
actor.SetMapper(mapper)

# Renderer
renderer = vtk.vtkRenderer()
renderer.AddActor(actor)
renderer.SetBackground(0.1, 0.2, 0.3)  # Background color

text_actor = vtk.vtkTextActor()
text_actor.SetInput("")                        # you can have multi-line text
prop = text_actor.GetTextProperty()
prop.SetFontSize(28)
prop.SetFontFamilyToCourier()
prop.SetColor(1.0, 1.0, 0.0)
prop.SetBackgroundColor(0.0, 0.2, 0.5)  # set the background color
prop.SetBackgroundOpacity(1.0)          # 1.0 = opaque, 0.0 = transparent
#prop.SetBackgroundRGBA(0.0, 0.2, 0.5, 0.5)  # alternative
prop.FrameOn()                          # bounding box
text_actor.SetPosition(50, 50)
renderer.AddActor2D(text_actor)

# Set up the camera to visualize the grid from the side
camera = vtk.vtkCamera()
camera.SetPosition(grid_size * 2, grid_size // 2, grid_size // 2)  # Positioned to the side
camera.SetFocalPoint(center[0], center[1], center[2])
camera.Roll(-90)  # Rotate the camera 90 degrees around the z-axis

renderer.SetActiveCamera(camera)

# Render window
render_window = vtk.vtkRenderWindow()
render_window.AddRenderer(renderer)
render_window.SetSize(1920, 1080) 
render_window.SetPosition(screen_width // 2, screen_height // 2)

# Render window interactor
render_window_interactor = vtk.vtkRenderWindowInteractor()
render_window_interactor.SetRenderWindow(render_window)

# Constants for recombination process
E_total = 4.184e15  # Energy in joules (1 megaton TNT equivalent)
m_p = 1.6726219e-27  # Mass of a proton in kg
m_e = 9.10938356e-31  # Mass of an electron in kg
c = 3e8  # Speed of light in m/s
eta = 0.0005  # Recombination efficiency
k = 1e-16  # Recombination rate constant
t_end = 10e-8  # Simulation end time in seconds (10 nanoseconds)

# Initial number of protons and electrons
N_p0 = E_total / (m_p * c**2)
N_e0 = E_total / (m_e * c**2)

# Define the differential equations for recombination and energy
def recombination_ode(N_p, N_e):
    dN_p_dt = -eta * k * N_p * N_e
    dN_e_dt = -eta * k * N_p * N_e
    dE_dt = -eta * k * N_p * N_e * (m_p * c**2 + m_e * c**2)
    
    return [dN_p_dt, dN_e_dt, dE_dt, num_recombined_particles]
    

total_recombined_particles = 0

# Function to update the visualization
def update_vtk(frame):
    global total_recombined_particles, particles, points, vertices, colors

    # Get active particles
    active_particles = np.argwhere(particles == 1)

    # Propagate particles outward to form a mushroom cloud
    for particle in active_particles:
        x, y, z = particle
        if z < len(particles[x, y]) - 1:
            particles[x, y, z + 1] = 1  # Move upward
            if z > center[2]:
                if x > 0: particles[x - 1, y, z] = 1  # Spread outward
                if x < len(particles) - 1: particles[x + 1, y, z] = 1
                if y > 0: particles[x, y - 1, z] = 1
                if y < len(particles) - 1: particles[x, y + 1, z] = 1

    # Clear previous points
    points.Reset()
    vertices.Reset()
    colors.Reset()

    recombined_particles = recombination_ode(len(active_particles), len(active_particles))

    new_shape = (len(particles) - recombined_particles, len(particles) - recombined_particles, len(particles) - recombined_particles)
    particles = particles[:new_shape[0], :new_shape[1], :new_shape[2]]

    if len(particles) != 0:
        # Convert particle positions to 3D point cloud
        active_points = np.argwhere(particles)

        total_recombined_particles += recombined_particles
        

        # Update the points in the VTK object
        for point in active_points:
            id = points.InsertNextPoint(point[0], point[1], point[2])
            vertices.InsertNextCell(1)
            vertices.InsertCellPoint(id)
            colors.InsertNextTuple3(255, 0, 0)  # Color influenced by neutralized energy
        
        # Update polydata
        polydata.Modified()

    text_actor.SetInput("Recombined particles: {}\nActive particles: {}".format(total_recombined_particles, len(particles)))

    if total_recombined_particles == len(particles):
        particles = np.zeros((total_recombined_particles, total_recombined_particles, total_recombined_particles), dtype=int)

    # Render the updated scene
    render_window.Render() 

# Timer callback for animation
def timer_callback(obj, event):
    global current_frame
    update_vtk(current_frame)
    current_frame += 1
    if current_frame >= num_frames:
        render_window_interactor.DestroyTimer(timer_id)

# Initialize frame counter
current_frame = 0

# Set up the animation timer
render_window_interactor.AddObserver('TimerEvent', timer_callback)
timer_id = render_window_interactor.CreateRepeatingTimer(100)

# Start the interaction and timer
render_window.Render()
render_window_interactor.Start()
