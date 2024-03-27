import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go
import tkinter as tk
from tkinter import ttk
from tqdm import tqdm
import random

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

def visualize_robot_arm_angles(isPlotting = True):
    global base_d, shoulder_d, elbow_d, wrist_pitch_d, base_length, shoulder_length, elbow_length, wrist_pitch_length

    # Convert degrees to radians
    base = degrees_to_radians(base_d)
    shoulder = degrees_to_radians(shoulder_d)
    elbow = degrees_to_radians(elbow_d)
    wrist_pitch = degrees_to_radians(wrist_pitch_d)

    # Create the 3D plot
    if isPlotting:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    # Plot the arm segments
    angles = [shoulder, elbow, wrist_pitch] # initial angles
    lengths = [shoulder_length, elbow_length, wrist_pitch_length] # initial lengths
    colors = ['g-', 'b-', 'm-'] # colors per segment used
    previous_point = (0, 0, 0) # initial point (so at base)
    previous_angle = 0 # initial angle (so at base)
    next_point = (np.cos(base) * base_length, np.sin(base) * base_length, 0)
    if isPlotting: plot_arm_segment(ax, previous_point, next_point, 'r-') # plot the base angle rotation according to a rotated line around the z axis
    unrotated_points = []

    # calculate the rotations for each joint with an unrotated base
    for angle, length, color in zip(angles, lengths, colors):
        angle += previous_angle
        next_point = get_next_point(previous_point, angle, length)
        #plot_arm_segment(ax, previous_point, next_point, color)
        unrotated_points.append(next_point)
        previous_angle = angle
        previous_point = next_point
    
    # plot the arm segments with the base rotation
    previous_point = (0, 0, 0)
    for point, length, color in zip(unrotated_points, lengths, colors):
        rotated_point = rotate_around_z(base, point)
        if isPlotting: plot_arm_segment(ax, previous_point, rotated_point, color)
        previous_point = rotated_point

    if isPlotting:
        # Add 3 control points points at specified locations
        point1 = (300, 0, 0)
        point2 = (0, 300, 0)
        point3 = (0, 0, 300)

        # Plot the additional points
        ax.scatter(*point1, color='r')
        ax.scatter(*point2, color='g')
        ax.scatter(*point3, color='b')

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

    return previous_point

def visualize_singularity(isStochastic = False):
    global base_d, shoulder_d, elbow_d, wrist_pitch_d, base_length, shoulder_length, elbow_length, wrist_pitch_length

    base_d = -180
    shoulder_d = 0
    elbow_d = 90
    wrist_pitch_d = 90
    no_base_rotate = [[], []]
    base_rotate = [[], [], []]

    if not isStochastic:
        # go over every possible angle for every joint with a slice without rotating the base
        print("getting 2D singularity")
        for shoulder_d in tqdm(range(0, 181, 2)):
            for elbow_d in range(90, -91, -5):
                for wrist_pitch_d in range(90, -91, -5):
                    # Convert degrees to radians
                    # print(f"{shoulder_d} {elbow_d}")
                    point = visualize_robot_arm_angles(False)
                    no_base_rotate[0].append(point[0])
                    no_base_rotate[1].append(point[2])

        print("getting 3D singularity")
        # go over every possible angle for every joint with a rotated base
        for base_d in tqdm(range(-180, 180, 5)):
            for shoulder_d in range(0, 181, 10):
                for elbow_d in range(90, -91, -10):
                    for wrist_pitch_d in range(90, -91, -10):
                        # Convert degrees to radians
                        # print(f"{shoulder_d} {elbow_d}")
                        point = visualize_robot_arm_angles(False)
                        base_rotate[0].append(point[0])
                        base_rotate[1].append(point[1])
                        base_rotate[2].append(point[2])
                        # print(f"b:\t{base_d} - \ts:\t{shoulder_d} - \te:\t{elbow_d} - \tw:\t{wrist_pitch_d} === {point}")
            

    else:
        random.seed(1925678)
        sample_size = 100000
        for sample in tqdm(range(0, sample_size)):
            base_d = -180
            shoulder_d = random.random() * 180
            elbow_d = (random.random() * 180) - 90
            wrist_pitch_d = (random.random() * 180) - 90

            point = visualize_robot_arm_angles(False)
            no_base_rotate[0].append(point[0])
            no_base_rotate[1].append(point[2])

            base_d = (random.random() * 360) - 180
            shoulder_d = random.random() * 180
            elbow_d = (random.random() * 180) - 90
            wrist_pitch_d = (random.random() * 180) - 90

            point = visualize_robot_arm_angles(False)
            base_rotate[0].append(point[0])
            base_rotate[1].append(point[1])
            base_rotate[2].append(point[2])

    # create plotly plot
    fig1 = go.Figure()
    fig2 = go.Figure()
    fig1.add_trace(
        go.Scatter(
            x=no_base_rotate[0],
            y=no_base_rotate[1],
            mode='markers',
            opacity=0.5,
            marker=dict(size=5)
        )
    )
    fig2.add_trace(
        go.Scatter3d(
                x=base_rotate[0], 
                y=base_rotate[1], 
                z=base_rotate[2], 
                mode='markers',
                opacity=0.1,
                marker=dict(size=1)
            )
        )

    # add plot limits
    fig1.update_layout(scene=dict(xaxis=dict(range=[-600, 600]), yaxis=dict(range=[-400, 800])))
    fig2.update_layout(scene=dict(xaxis=dict(range=[-600, 600]), yaxis=dict(range=[-600, 600]), zaxis=dict(range=[-400, 800])))
    # Show the plot
    fig1.show()  
    fig2.show()

                

# Example usage
# Create a tkinter window
window = tk.Tk()
window.title("Angle Visualizer")

# Define the initial angles
base_d = 0
shoulder_d = 157.50
elbow_d = -180
wrist_pitch_d = 0

# Define the segment lengths
base_length = 100
shoulder_length = 200
elbow_length = 200
wrist_pitch_length = 100

# Define a function to update the angles
def update_angles(_):
    global base_d, shoulder_d, elbow_d, wrist_pitch_d
    base_d = base_slider.get()
    shoulder_d = shoulder_slider.get()
    elbow_d = elbow_slider.get()
    wrist_pitch_d = wrist_pitch_slider.get()
    print("base, shoulder, elbow, wrist_pitch")
    print(base_d, shoulder_d, elbow_d, wrist_pitch_d)
    plt.close('all')

def visualise_sing():
    visualize_singularity()

def visualise_sing_stoch():
    visualize_singularity(True)
    

# Create sliders for each angle
base_slider = ttk.Scale(window, from_=-180, to=180, length=200, command=update_angles)
base_slider.set(base_d)
base_slider.pack()

shoulder_slider = ttk.Scale(window, from_=0, to=180, length=200, command=update_angles)
shoulder_slider.set(shoulder_d)
shoulder_slider.pack()

elbow_slider = ttk.Scale(window, from_=90, to=-90, length=200, command=update_angles)
elbow_slider.set(elbow_d)
elbow_slider.pack()

wrist_pitch_slider = ttk.Scale(window, from_=90, to=-90, length=200, command=update_angles)
wrist_pitch_slider.set(wrist_pitch_d)
wrist_pitch_slider.pack()

# Create a button to visualize the robot arm
visualize_button = ttk.Button(window, text="Visualize", command=visualize_robot_arm_angles)
visualize_button.pack()

# Create a button to visualize the robot arm
visualize_button = ttk.Button(window, text="Singularity", command=visualise_sing)
visualize_button.pack()

# Create a button to visualize the robot arm
visualize_button = ttk.Button(window, text="Singularity (stochastic)", command=visualise_sing_stoch)
visualize_button.pack()

# Run the tkinter event loop
window.mainloop()

# visualize_robot_arm(base, shoulder, elbow, wrist_pitch)