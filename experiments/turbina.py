import open3d as o3d
import numpy as np

# Parameters
fn = 100  # Smoothness
shaft_length = 200  # Length of the central shaft
shaft_radius = 2  # Radius of the shaft

# Parameters for blades in each compressor stage
compressor_stage_count = 9
blade_counts = [18, 16, 14, 12, 10, 8, 6, 4, 2]  # Number of blades per stage (can be varied)
blade_lengths = [20, 18, 16, 14, 12, 10, 8, 6, 4]  # Length of blades per stage (descends)
blade_height = 5  # Height of blades

# Turbine parameters
# Parameters for turbine blades
turbine_blade_counts = [18, 16, 14, 12]  # Number of blades per stage (can be varied)
turbine_blade_lengths = [20, 18, 16, 14]  # Length of blades per stage (descends)

turbine_blade_count = 24  # Set to four exhaust blades
turbine_blade_length = 30  # Length of the turbine blades
turbine_blade_height = turbine_blade_length / 1.5  # Height of the turbine blades, similar to the compressor blade height
turbine_stage_distance_from_front = 120  # Distance from the front end of the shaft to the turbine stage

# Main central fan blade parameters
main_fan_blade_count = 24  # Number of fan blades
main_fan_blade_length = 15  # Length of the fan blades
main_fan_diameter = 40  # Diameter of the fan

nacelle_thickness = 16  # Thickness of the nacelle or shell
engine_diameter = 80  # Overall diameter of the engine nacelle

show_shell = True  # Boolean to toggle shell visibility

# Additional Parameters for the shape of the nacelle and chambers
combustor_length = 30  # Length of the combustion chamber
exhaust_diameter = turbine_blade_length  # Diameter at the exhaust

# Construct the central shaft
def shaft(total_length):
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=shaft_radius, height=total_length, resolution=fn)
    # Centering the shaft along the length
    mesh.translate([0, 0, total_length / 2])  # Adjusts the position based on the new total length
    return mesh

def calculate_engine_length():
    # Length of compressor and turbine stages with assumed spacing
    compressor_total_length = sum([blade_height + 2 for _ in blade_counts])  # Blade height plus spacing
    turbine_total_length = sum([turbine_blade_height / 1.5 for _ in turbine_blade_counts])  # Blade height with spacing

    # Total engine length might include other components like space for nacelle, fan assembly etc.
    total_length = compressor_total_length + turbine_total_length + turbine_stage_distance_from_front

    return total_length

# Construct compressor stages
def compressor_stages():
    meshes = []
    for stage, blade_count in enumerate(blade_counts):
        for i in np.linspace(0, 359, blade_count, endpoint=False):
            rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.radians(i)))
            rotation_matrix = np.vstack((rotation_matrix, [0, 0, 0]))  # Add a row at the bottom
            rotation_matrix = np.hstack((rotation_matrix, [[0], [0], [0], [1]]))  # Add a column on the right
            translation_matrix = np.eye(4)
            translation_matrix[:3, 3] = [0, 0, stage * (blade_height + 2) - 1]
            transform_matrix = np.dot(rotation_matrix, translation_matrix)  # Use np.dot for matrix multiplication
            blade_mesh = compressor_blade(blade_lengths[stage], blade_height)
            blade_mesh.transform(transform_matrix)
            meshes.append(blade_mesh)
    return meshes

def compressor_blade(length, height):
    root_width = length  # Base width of the blade
    tip_width = root_width * 0.5  # Tip width is half of the root width

    # Blade profile vertices
    vertices = np.array([
        [0, 0, 0],  # Bottom left
        [root_width, 0, 0],  # Bottom right
        [tip_width, height * 0.9, 0],  # Near the top right, but not the very tip
        [0, height, 0]  # Top left
    ])

    # Creating faces
    faces = np.array([
        [0, 1, 2],
        [0, 2, 3],
        [2, 1, 0],  # Mirrored face
        [3, 2, 0]   # Mirrored face
    ])

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()  # To ensure the mesh is rendered smoothly

    return mesh


# Construct main central fan blades
def main_fan_blades():
    meshes = []
    for i in np.linspace(0, 359, main_fan_blade_count, endpoint=False):
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.radians(i)))
        rotation_matrix = np.vstack((rotation_matrix, [0, 0, 0]))  # Add a zero row at the bottom
        rotation_matrix = np.hstack((rotation_matrix, [[0], [0], [0], [1]]))  # Add a one in the bottom right corner
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = [shaft_radius + 1, 0, shaft_length / 2 - main_fan_blade_length]
        transform_matrix = np.dot(rotation_matrix, translation_matrix)  # Use np.dot or @ for matrix multiplication
        blade_mesh = fan_blade()
        blade_mesh.transform(transform_matrix)
        meshes.append(blade_mesh)
    return meshes

def fan_blade():
    height = main_fan_diameter
    root_width = 3  # Width at the root of the blade
    tip_width = 1.5  # Width at the tip of the blade, narrower than the root

    # Blade profile vertices
    vertices = np.array([
        [0, 0, 0],  # Bottom left
        [root_width, 0, 0],  # Bottom right
        [tip_width, height * 0.9, 0],  # Near the top right, but not the very tip
        [0, height, 0]  # Top left
    ])

    # Creating faces
    # Creating faces and their mirrored counterparts
    faces = np.array([
        [0, 1, 2],
        [0, 2, 3],
        [2, 1, 0],  # Mirrored face
        [3, 2, 0]   # Mirrored face
    ])

    # Create mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()  # To ensure the mesh is rendered smoothly

    return mesh

# Construct turbine stages
def turbine_stages():
    meshes = []
    for stage, blade_count in enumerate(turbine_blade_counts):
        for i in np.linspace(0, 359, blade_count, endpoint=False):
            # Generate the 3x3 rotation matrix
            rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.radians(i)))
            # Convert to 4x4 by adding a row and a column
            rotation_matrix = np.vstack((rotation_matrix, [0, 0, 0]))
            rotation_matrix = np.hstack((rotation_matrix, [[0], [0], [0], [1]]))
            # Create a 4x4 translation matrix
            translation_matrix = np.eye(4)
            translation_matrix[:3, 3] = [0, 0, turbine_stage_distance_from_front + stage * (turbine_blade_height / 1.5) - 1]
            # Matrix multiplication to create a transformation matrix
            transform_matrix = rotation_matrix @ translation_matrix
            blade_mesh = turbine_blade(turbine_blade_lengths[stage], turbine_blade_height)
            blade_mesh.transform(transform_matrix)
            meshes.append(blade_mesh)
    return meshes

def turbine_blade(length, height):
    root_width = length / 2
    tip_width = root_width * 2

    # Blade profile vertices
    vertices = np.array([
        [0, 0, 0],  # Root of the blade at the shaft
        [root_width, 0, 0],  # Outer edge of the blade root
        [tip_width, height, 0],  # Outer edge of the blade tip
        [0, height, 0]  # Tip of the blade at the shaft
    ])

    # Creating faces
    faces = np.array([
        [0, 1, 2],
        [0, 2, 3],
        [2, 1, 0],  # Mirrored face
        [3, 2, 0]   # Mirrored face
    ])

    # Create mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()  # To ensure the mesh is rendered smoothly

    return mesh


def create_tapered_section(height, outer_radius, inner_radius, resolution=100):
    bottom_radius = outer_radius
    top_radius = inner_radius
    # Create a cylinder that will be tapered to form the conical frustum
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=bottom_radius, height=height, resolution=resolution)
    vertices = np.asarray(mesh.vertices)
    
    # Calculate the scale factor for each vertex to create the tapering effect
    for i in range(len(vertices)):
        # Interpolation factor (0 at base, 1 at top)
        t = (vertices[i, 2] + height / 2) / height
        scale_factor = (1 - t) * bottom_radius + t * top_radius
        vertices[i, 0] *= scale_factor / bottom_radius
        vertices[i, 1] *= scale_factor / bottom_radius

    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.compute_vertex_normals()

    # Translate the mesh so that its base is on the XY plane
    mesh.translate([0, 0, -height / 2])
    return mesh
def create_hollow_section(height, outer_radius, inner_radius, resolution=100):
    # Create outer shell
    outer_shell = o3d.geometry.TriangleMesh.create_cylinder(radius=outer_radius, height=height, resolution=resolution)
    outer_shell.compute_vertex_normals()

    # Create inner shell
    inner_shell = o3d.geometry.TriangleMesh.create_cylinder(radius=inner_radius, height=height, resolution=resolution)
    inner_shell.compute_vertex_normals()

    # Translate shells so their bases align at the origin
    outer_shell.translate([0, 0, -height / 2])
    inner_shell.translate([0, 0, -height / 2])

    # Flip the inner shell faces to point inward
    inner_shell.triangles = o3d.utility.Vector3iVector(np.asarray(inner_shell.triangles)[:, [2, 1, 0]])
    inner_shell.compute_vertex_normals()

    # Flip the inner shell faces to point inward
    num_vertices = len(outer_shell.vertices)
    cap_triangles = set(range(resolution)) | set(range(num_vertices - resolution, num_vertices))

    def filter_caps(triangles, cap_indices):
        return [triangle for i, triangle in enumerate(triangles) if not set(triangle).intersection(cap_indices)]

    inner_shell.triangles = o3d.utility.Vector3iVector(filter_caps(np.asarray(inner_shell.triangles), cap_triangles))

    # Combine the outer shell and the inner shell into one mesh
    hollow_section = inner_shell

    # Recompute normals for the hollow section
    hollow_section.compute_vertex_normals()

    return hollow_section


def assemble_nacelle():
    cone_length = shaft_length * 0.4  # Length of the forward cone
    cylinder_length = combustor_length  # Middle cylinder section
    back_cone_length = shaft_length - cone_length - cylinder_length  # Length of the backwards cone

    # Create hollow sections
    front_section = create_hollow_section(height=cone_length, outer_radius=turbine_blade_length, inner_radius=turbine_blade_length - 1)
    middle_section = create_hollow_section(height=cylinder_length, outer_radius=main_fan_diameter, inner_radius=main_fan_diameter - 1)
    rear_section = create_hollow_section(height=back_cone_length, outer_radius=turbine_blade_lengths[0], inner_radius=turbine_blade_lengths[0] - 1)

    # Position the sections correctly along the Z-axis
    middle_section.translate([0, 0, -cone_length])  # Adjust based on the height of the front section
    rear_section.translate([0, 0, -cone_length + -cylinder_length])  # Adjust based on the combined heights of the front and middle sections

    # Merge the meshes to create a single nacelle mesh
    nacelle = front_section + middle_section + rear_section

    nacelle.translate([0, 0, back_cone_length + cone_length + cylinder_length])
    return nacelle


def create_cone(height, base_radius, top_radius):
    """Creates an approximated cone using a cylinder by scaling."""
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=base_radius, height=height, resolution=fn)
    # Uniform scale first
    mesh.scale(base_radius, center=np.array([0, 0, 0]))
    # Manually adjust vertices to create a tapered effect
    vertices = np.asarray(mesh.vertices)
    # Linearly interpolate radii along the height
    for i in range(len(vertices)):
        factor = 1 - ((vertices[i][2] + height / 2) / height)  # Normalize height to range [0, 1]
        current_radius = base_radius + factor * (top_radius - base_radius)
        vertices[i][0] *= current_radius / base_radius
        vertices[i][1] *= current_radius / base_radius
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.compute_vertex_normals()
    return mesh

def create_cylinder(height, base_radius, top_radius=None):
    """Creates a cylinder and scales it uniformly based on the base radius."""
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=height, resolution=fn)
    mesh.scale(base_radius, center=np.array([0, 0, -height / 2]))  # Scale uniformly using base radius
    return mesh

# Assemble the jet engine
def jet_engine():
    meshes = []
    # Calculate total engine length
    total_length = calculate_engine_length()

    # Create shaft with adjusted length
    shaft_mesh = shaft(total_length)
    shaft_mesh.paint_uniform_color([0.7, 0.7, 0.7])  # Silver color

    # Append all blade meshes
    compressor_meshes = compressor_stages()
    for mesh in compressor_meshes:
        mesh.paint_uniform_color([0.7, 0.7, 1])  # Light blue color
        shaft_mesh += mesh  # Append directly to the shaft mesh

    fan_meshes = main_fan_blades()
    for mesh in fan_meshes:
        mesh.paint_uniform_color([0, 0, 1])  # Blue color
        shaft_mesh += mesh  # Append directly to the shaft mesh

    turbine_meshes = turbine_stages()
    for mesh in turbine_meshes:
        mesh.paint_uniform_color([1, 0.5, 0])  # Orange color
        shaft_mesh += mesh  # Append directly to the shaft mesh

    # Create and add the nacelle if toggled
    if show_shell:
        nacelle_mesh = assemble_nacelle()
        meshes.append(nacelle_mesh)

    return meshes, shaft_mesh


def visualize_jet_engine():
    engine_parts, shaft_mesh = jet_engine()

    def rotate_geometry(vis):
        """Animation callback to rotate the shaft."""
        rotation_angle = np.radians(6)  # Rotate the shaft by 10 degrees each frame
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, rotation_angle))
        shaft_mesh.rotate(rotation_matrix, center=(0, 0, 0))
        vis.update_geometry(shaft_mesh)
        return True

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for part in engine_parts:
        vis.add_geometry(part)

    # Add the shaft separately if not included in engine_parts
    if shaft_mesh not in engine_parts:
        vis.add_geometry(shaft_mesh)

    vis.register_animation_callback(rotate_geometry)

    vis.run()
    vis.destroy_window()

# Execute the visualization
visualize_jet_engine()
