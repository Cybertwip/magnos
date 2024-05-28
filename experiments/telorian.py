import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

class Car:
    def __init__(self):
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.25, 0.0])  # Start position above the ground
        self.mass = 1000  # Assume mass of car is 1000 kg
        self.acceleration = self.calculate_acceleration(1.21e9, 864.29)
        self.car = self.create_car()
        self.road = self.create_road()

    def create_car(self):
        car = o3d.geometry.TriangleMesh.create_box(width=1.0, height=0.5, depth=0.3)
        car.paint_uniform_color([0.1, 0.1, 0.7])
        car.translate(self.position)  # Initial position
        return car

    def create_road(self):
        road = o3d.geometry.TriangleMesh.create_box(width=20000000.0, height=0.1, depth=2.0)
        road.paint_uniform_color([0.2, 0.2, 0.2])
        road.translate([0.0, 0.0, -1.0])  # Position the road below the car
        return road

    def calculate_acceleration(self, power, current):
        voltage = 1.4e6  # 1.4 million volts
        force = (power / voltage) * current  # Force in newtons
        acceleration = force / self.mass  # Acceleration in m/s^2
        return acceleration

    def move(self, direction):
        regular_acceleration = 2.0  # Regular car acceleration in m/s^2
        if direction == 'W':
            self.velocity[0] += regular_acceleration
        elif direction == 'S':
            self.velocity[0] -= regular_acceleration
        self.position += self.velocity
        self.car.translate(self.velocity)
        self.state = "0 gigawatts"

    def time_travel(self):
        self.velocity += self.acceleration  # Apply significant acceleration for time travel
        self.state = "1.21 gigawatts"

    def reset_position(self):
        self.position = np.array([0.0, 0.25, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.car.translate(-self.car.get_center() + self.position)

class Viewer3D:
    def __init__(self, title="3D Visualization"):
        self.app = o3d.visualization.gui.Application.instance
        self.app.initialize()

        self.window = self.app.create_window(title, width=800, height=600)
        self.widget = o3d.visualization.gui.SceneWidget()
        self.widget.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self.main_vis = self.widget.scene

        self.key_states = {'W': False, 'A': False, 'S': False, 'D': False, 'U': False}

        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self.widget)

        self.setup_gui()
        self.setup_scene()

        self.window.set_on_key(self._on_key_event)

    def _on_key_event(self, event):
        if event.key == gui.KeyName.W:
            self.key_states['W'] = event.type == gui.KeyEvent.Type.DOWN
        elif event.key == gui.KeyName.S:
            self.key_states['S'] = event.type == gui.KeyEvent.Type.DOWN
        elif event.key == gui.KeyName.A:
            self.key_states['A'] = event.type == gui.KeyEvent.Type.DOWN
        elif event.key == gui.KeyName.D:
            self.key_states['D'] = event.type == gui.KeyEvent.Type.DOWN
        elif event.key == gui.KeyName.U:
            self.key_states['U'] = event.type == gui.KeyEvent.Type.DOWN

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget.frame = r
        width = 17 * layout_context.theme.font_size
        height = min(r.height, self.vert.calc_preferred_size(layout_context, gui.Widget.Constraints()).height)
        self.vert.frame = gui.Rect(r.get_right() - width, r.y, width, height)

    def setup_gui(self):
        self.vert = gui.Vert(0, gui.Margins(10))
        self.speed_label = gui.Label("Speed: 0 m/s")
        self.state_label = gui.Label("State: ")
        self.reset_button = gui.Button("Reset Position")
        self.reset_button.set_on_clicked(self.reset_position)
        self.vert.add_child(self.speed_label)
        self.vert.add_child(self.reset_button)
        self.window.add_child(self.vert)

    def reset_position(self):
        self.car.reset_position()
        self.update_gui()

    def get_default_material(self):
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultLit"  # Use a simple lit shader
        return material

    def setup_scene(self):
        default_material = self.get_default_material()
        self.car = Car()
        self.main_vis.add_geometry("Road", self.car.road, default_material)
        self.main_vis.add_geometry("Car", self.car.car, default_material)
        return self.car

    def update_car(self):
        for key, pressed in self.key_states.items():
            if pressed:
                if key == 'U':
                    self.car.time_travel()
                else:
                    self.car.move(key)
        car_pos = self.car.car.get_center()
        eye = car_pos + np.array([5.0, 3.0, 5.0])
        center = car_pos
        up = np.array([0.0, 1.0, 0.0])
        self.main_vis.camera.look_at(center, eye, up)

    def update_gui(self):
        speed = np.linalg.norm(self.car.velocity)
        self.speed_label.text = f"Speed: {speed:.2f} m/s"
        self.main_vis.remove_geometry("Car")
        self.main_vis.add_geometry("Car", self.car.car, self.get_default_material())

    def run_one_tick(self):
        self.update_car()
        self.update_gui()
        gui.Application.instance.post_to_main_thread(self.window, lambda: None)
        return self.app.run_one_tick()

if __name__ == "__main__":
    viewer = Viewer3D("3D Dynamic Scene")
    while True:
        if not viewer.run_one_tick():
            break
