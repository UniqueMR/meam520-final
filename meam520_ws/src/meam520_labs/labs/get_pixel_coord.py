import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Load the image
image_path = "side_camera_rgb.png"  # Replace "your_image.png" with the path to your image
img = mpimg.imread(image_path)

# Display the image
fig, ax = plt.subplots()
ax.imshow(img)
plt.axis('on')

# List to store clicked points
clicked_points = []

# Function to handle mouse clicks
def onclick(event):
    if event.button == 1:  # Left mouse button
        x = int(event.xdata)
        y = int(event.ydata)
        print(f"Clicked at pixel coordinates: ({x}, {y})")
        clicked_points.append((x, y))

# Connect the onclick function to the figure
cid = fig.canvas.mpl_connect('button_press_event', onclick)

# Display the plot
plt.show()

# Print the list of clicked points
print("Clicked points:", clicked_points)
