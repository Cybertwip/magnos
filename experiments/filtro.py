import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui

import time
import random


def pumice_filtration(initial_concentration, adsorption_efficiency, water_volume, pumice_volume):
    if water_volume <= 0:
        print("No water volume available for filtration. Returning initial concentration.")
        return initial_concentration, initial_concentration  # Return initial as final if no water is present

    effective_adsorption = adsorption_efficiency * (pumice_volume / water_volume)
    final_concentration = initial_concentration * (1 - effective_adsorption)
    return initial_concentration, final_concentration

class Viewer3D:
    def __init__(self, title="3D Visualization"):
        self.app = o3d.visualization.gui.Application.instance
        self.app.initialize()


        self.window = self.app.create_window(title, width=800, height=600)


        self.widget = o3d.visualization.gui.SceneWidget()
        self.widget.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        #self.widget.setup_camera(60, self.widget.scene.bounding_box, [0, 0, 0])

        self.main_vis = self.widget.scene

        bg_image = o3d.io.read_image("background.jpg")

        # Set the background image
        self.main_vis.set_background([0, 1, 1, 1], bg_image)

        self.main_vis.show_skybox(False)
        self.fire_particles = []

        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self.widget)

        self.setup_gui()

        self.setup_scene()
        self.last_time = time.time()
        self.update_interval = 0.001  # very fast updates
        self.pause_time = 0.001  # short pause for visibility
        self.is_paused = False
        self.pause_start_time = None
   
        # Simulation parameters
        self.initial_concentration = 100  # mg/L
        self.adsorption_efficiency = 0.7  # 70%
    
    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget.frame = r
        width = 17 * layout_context.theme.font_size
        height = min(r.height, self.vert.calc_preferred_size(layout_context, gui.Widget.Constraints()).height)
        self.vert.frame = gui.Rect(r.get_right() - width, r.y, width, height)


    def setup_gui(self):
        self.vert = o3d.visualization.gui.Vert(0, o3d.visualization.gui.Margins(10))
        self.old_pollution_label = o3d.visualization.gui.Label("Contaminación inicial: 0 mg/L")
        self.new_pollution_label = o3d.visualization.gui.Label("Contaminación final: 0 mg/L")
        self.vert.add_child(self.old_pollution_label)
        self.vert.add_child(self.new_pollution_label)
        self.window.add_child(self.vert)

    def get_default_material(self):
        # Create a simple material with default settings
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultLit"  # Use a simple lit shader
        return material

    def update_gui(self, old_pollution, new_pollution):
        self.old_pollution_label.text = f"Contaminación inicial: {old_pollution:.2f} mg/L"
        self.new_pollution_label.text = f"Contaminación final: {new_pollution:.2f} mg/L"

    def setup_scene(self):
        self.path = self.create_s_curve(np.array([-1, 0, 1]), np.array([1, 0, -1]), vertical_offset=0.5, points=300)

        default_material = self.get_default_material()  # Get the default material

        self.pumice_stones = [self.create_pumice_stone(self.path[i], 0.05) for i in np.linspace(10, len(self.path) - 50, 20, dtype=int)]
        for i, (pumice, size) in enumerate(self.pumice_stones):
            self.main_vis.add_geometry(f"pumice_{i}", pumice, default_material)

        min_bound = np.min(self.path, axis=0)
        max_bound = np.max(self.path, axis=0)
        self.box = self.create_transparent_box([min_bound, max_bound], [0.85, 0.85, 0.85])
        self.main_vis.add_geometry("bounding_box", self.box, default_material)

        self.flow_radius = 0.1  # Reduced radius for visual clarity
        self.water_flow = {}
        self.flow_length = 80
        self.start_index = 0
        self.init_fire()
    
    def init_fire(self):
        # Calculate positions for fire spheres on the top face of the bounding box
        max_bound = self.box.get_max_bound()
        min_bound = self.box.get_min_bound()
        side_length_x = max_bound[0] - min_bound[0]
        side_length_y = max_bound[1] - min_bound[1]
        num_rows = 2
        num_cols = 3
        self.fire_particles = []
        default_material = self.get_default_material()  # Get the default material

        for i in range(num_rows):
            for j in range(num_cols):
                x_pos = min_bound[0] + side_length_x * (j + 0.5) / num_cols
                y_pos = min_bound[1] + side_length_y * (i + 0.5) / num_rows
                position = np.array([x_pos, y_pos, max_bound[2]])

                particle = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
                particle.translate(position)
                particle.paint_uniform_color([1, 0.4, 0])  # Fire-like color
                idx = i * num_cols + j
                self.fire_particles.append((particle, 0.05))  # Store initial radius as well
                self.main_vis.add_geometry(f"fire_particle_{idx}", particle, default_material)

    def update_fire(self):
        default_material = self.get_default_material()  # Get the default material

        # Toggle visibility of fire particles to simulate flickering
        for i, (particle, init_radius) in enumerate(self.fire_particles):
            visible = random.choice([True, False])  # Randomly choose visibility to simulate flickering
            self.main_vis.remove_geometry(f"fire_particle_{i}")
            if visible:
                self.main_vis.add_geometry(f"fire_particle_{i}", particle, default_material)



    def create_pumice_stone(self, position, size):
        scatter = np.random.normal(scale=0.10, size=3)
        pumice = o3d.geometry.TriangleMesh.create_sphere(radius=size)
        pumice.translate(position + scatter)
        pumice.paint_uniform_color([0.3, 0.3, 0.3])
        return (pumice, size)  # Return both the mesh and the size

    def create_s_curve(self, start, end, points=300, vertical_offset=0.5):
        x = np.linspace(start[0], end[0], points)
        y = np.sin(x * np.pi) * vertical_offset + start[1]
        z = np.linspace(start[2], end[2], points) * 2
        return np.vstack((x, y, z)).T

    def create_transparent_box(self, corners, color=[0.8, 0.8, 0.8]):
        box = o3d.geometry.LineSet()
        points = [
            [corners[0][0], corners[0][1], corners[0][2]],
            [corners[1][0], corners[0][1], corners[0][2]],
            [corners[1][0], corners[1][1], corners[0][2]],
            [corners[0][0], corners[1][1], corners[0][2]],
            [corners[0][0], corners[0][1], corners[1][2]],
            [corners[1][0], corners[0][1], corners[1][2]],
            [corners[1][0], corners[1][1], corners[1][2]],
            [corners[0][0], corners[1][1], corners[1][2]]
        ]
        lines = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]
        box.points = o3d.utility.Vector3dVector(points)
        box.lines = o3d.utility.Vector2iVector(lines)
        box.colors = o3d.utility.Vector3dVector([color] * len(lines))
        return box

    
    def update_flow(self):
        default_material = self.get_default_material()  # Get the default material
        if self.is_paused:
            if time.time() - self.pause_start_time >= self.pause_time:
                self.is_paused = False
                self.start_index = 0  # Reset the start index for a new flow cycle
                # Clear existing flow geometries might be called prematurely, check conditions
                self.water_flow = {}
            return

        if time.time() - self.last_time > self.update_interval:
            self.update_fire()

            # Existing segment handling might be clearing too soon
            for key in list(self.water_flow.keys()):
                if key < self.start_index or key >= self.start_index + self.flow_length:
                    self.main_vis.remove_geometry(f"water_flow_{key}")
                    del self.water_flow[key]
                else:
                    new_pos = self.path[key]
                    self.water_flow[key][0].translate(new_pos - self.water_flow[key][0].get_center(), relative=False)

            # Segment creation logic
            for i in range(self.start_index, min(self.start_index + self.flow_length, len(self.path))):
                if i not in self.water_flow:
                    new_segment, volume = self.create_water_segment(self.path[i])
                    self.water_flow[i] = (new_segment, volume)
                    self.main_vis.add_geometry(f"water_flow_{i}", new_segment, default_material)
        
            # Calculate new pollution level based on pumice interaction

            total_water_volume = sum(volume for _, volume in self.water_flow.values())

            total_pumice_volume = sum([4/3 * np.pi * (radius ** 3) for _, radius in self.pumice_stones])
            old_pollution_level, new_pollution_level = pumice_filtration(
                self.initial_concentration,
                self.adsorption_efficiency,
                total_water_volume,
                total_pumice_volume
            )

            self.update_gui(old_pollution_level, new_pollution_level)

            self.start_index += 1
            self.last_time = time.time()

            if self.start_index >= len(self.path) + self.flow_length:
                self.is_paused = True
                self.pause_start_time = time.time()
   
   

    def create_water_segment(self, position):
        height = 0.05  # Height of each cylinder
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=self.flow_radius, height=height)
        cylinder.translate(position, relative=False)
        cylinder.paint_uniform_color([0, 0, 1])
        volume = np.pi * (self.flow_radius ** 2) * height  # Calculate volume
        return (cylinder, volume)
            
    def run_one_tick(self):
        self.update_flow()
        gui.Application.instance.post_to_main_thread(self.window, lambda: None)

        return self.app.run_one_tick()

if __name__ == "__main__":
    viewer = Viewer3D("3D Dynamic Scene")
    while True:
        if not viewer.run_one_tick():
            break
