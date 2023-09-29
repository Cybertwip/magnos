from PIL import Image
import random

# Load the height map image (replace 'height_map.png' with your file path)
height_map = Image.open('racing.png')


height_map = height_map.convert('L')

# Define the dimensions of the detail map (should match the height map)
detail_map_width, detail_map_height = height_map.size

# Create a blank detail map image with the same dimensions
detail_map = Image.new('L', (detail_map_width, detail_map_height))

# Define parameters for the detail map generation
detail_scale = 0.02  # Adjust the scale to control the intensity of the details

# Process each pixel in the detail map
for x in range(detail_map_width):
    for y in range(detail_map_height):
        # Calculate a noise value based on the pixel position
        noise = random.uniform(-1, 1) * detail_scale
        
        # Get the corresponding elevation value from the height map
        elevation = height_map.getpixel((x, y))
        
        # Add the noise to the elevation value
        new_elevation = min(255, max(0, elevation + int(noise * 255)))
        
        # Set the elevation value in the detail map image
        detail_map.putpixel((x, y), new_elevation)

# Save the detail map image (replace 'detail_map.png' with your desired output file path)
detail_map.save('racing_detail.png')

print("Detail map generation complete.")
