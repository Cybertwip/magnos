from PIL import Image

# Load the height map image (replace 'height_map.png' with your file path)
height_map = Image.open('racing.png')

height_map = height_map.convert('L')

# Define threshold values (adjust these as needed)
min_elevation = 0     # Minimum elevation for transparency
max_elevation = 255   # Maximum elevation for full opacity

# Create a blank alpha map image with the same dimensions
alpha_map = Image.new('L', height_map.size)

# Process each pixel in the height map
for x in range(height_map.width):
    for y in range(height_map.height):
        # Get the elevation value of the current pixel
        elevation = height_map.getpixel((x, y))
        
        # Calculate the alpha value based on elevation
        alpha = (elevation - min_elevation) / (max_elevation - min_elevation)
        alpha = max(0, min(255, int(alpha * 255)))  # Clamp alpha value between 0 and 255
        
        # Set the alpha value in the alpha map image
        alpha_map.putpixel((x, y), alpha)

# Save the alpha map image (replace 'alpha_map.png' with your desired output file path)
alpha_map.save('racing_alpha.png')

print("Alpha map generation complete.")
