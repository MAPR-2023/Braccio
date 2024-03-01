import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk

def degrees_to_radians(degrees):
    return degrees * np.pi / 180

def get_next_point(previous_point, angle, length):
    x1, y1, z1 = previous_point
    # calculate the non rotated point in a 2D XZ plane
    x2 = x1 + length * np.cos(angle)
    y2 = y1
    z2 = z1 + length * np.sin(angle)

    return (x2, y2, z2)

def rotate_around_z(base, point):
    x1, y1, z1 = point
    # rotate around the x axis
    x2 = x1 * np.cos(base) - y1 * np.sin(base)
    y2 = x1 * np.sin(base) + y1 * np.cos(base)
    z2 = z1
    return (x2, y2, z2)

def plot_arm_segment(ax, previous_point, next_point, color):
    x1, y1, z1 = previous_point
    x2, y2, z2 = next_point
    ax.plot([x1, x2], [y1, y2], [z1, z2], color)

def visualize_robot_arm_angles():
    global base, shoulder, elbow, wrist_pitch
    # Define the arm lengths
    base_length = 100
    shoulder_length = 200
    elbow_length = 200
    wrist_pitch_length = 100

    # Convert degrees to radians
    base = degrees_to_radians(base)
    shoulder = degrees_to_radians(shoulder)
    elbow = degrees_to_radians(elbow)
    wrist_pitch = degrees_to_radians(wrist_pitch)

    # Create the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the arm segments
    angles = [shoulder, elbow, wrist_pitch]
    lengths = [shoulder_length, elbow_length, wrist_pitch_length]
    colors = ['g-', 'b-', 'm-']
    previous_point = (0, 0, 0)
    previous_angle = 0
    next_point = (np.cos(base) * base_length, np.sin(base) * base_length, 0)
    plot_arm_segment(ax, previous_point, next_point, 'r-')
    unrotated_points = []

    for angle, length, color in zip(angles, lengths, colors):
        angle += previous_angle
        next_point = get_next_point(previous_point, angle, length)
        #plot_arm_segment(ax, previous_point, next_point, color)
        unrotated_points.append(next_point)
        previous_angle = angle
        previous_point = next_point
    
    previous_point = (0, 0, 0)
    for point, length, color in zip(unrotated_points, lengths, colors):
        rotated_point = rotate_around_z(base, point)
        plot_arm_segment(ax, previous_point, rotated_point, color)
        previous_point = rotated_point

    # Set the plot limits
    ax.set_xlim([-400, 400])
    ax.set_ylim([-400, 400])
    ax.set_zlim([0, 800])

    # Set the plot labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show the plot
    plt.show()

# Example usage
# Create a tkinter window
window = tk.Tk()
window.title("Angle Visualizer")

# Define the initial angles
base = 60
shoulder = 45
elbow = 30
wrist_pitch = 90

# Define a function to update the angles
def update_angles(_):
    global base, shoulder, elbow, wrist_pitch
    base = base_slider.get()
    shoulder = shoulder_slider.get()
    elbow = elbow_slider.get()
    wrist_pitch = wrist_pitch_slider.get()
    plt.close('all')
    

# Create sliders for each angle
base_slider = ttk.Scale(window, from_=-180, to=180, length=200, command=update_angles)
base_slider.set(base)
base_slider.pack()

shoulder_slider = ttk.Scale(window, from_=0, to=180, length=200, command=update_angles)
shoulder_slider.set(shoulder)
shoulder_slider.pack()

elbow_slider = ttk.Scale(window, from_=90, to=-90, length=200, command=update_angles)
elbow_slider.set(elbow)
elbow_slider.pack()

wrist_pitch_slider = ttk.Scale(window, from_=90, to=-90, length=200, command=update_angles)
wrist_pitch_slider.set(wrist_pitch)
wrist_pitch_slider.pack()

# Create a button to visualize the robot arm
visualize_button = ttk.Button(window, text="Visualize", command=visualize_robot_arm_angles)
visualize_button.pack()

# Run the tkinter event loop
window.mainloop()

# visualize_robot_arm(base, shoulder, elbow, wrist_pitch)