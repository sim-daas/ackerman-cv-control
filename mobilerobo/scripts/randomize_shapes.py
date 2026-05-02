#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import random
import math
import os

# Path to the world file
WORLD_PATH = '/home/ubuntu/githubrepos/ackerman-cv-control/mobilerobo/worlds/shapes.sdf'

def randomize_positions():
    if not os.path.exists(WORLD_PATH):
        print(f"Error: {WORLD_PATH} not found.")
        return

    # Parse the SDF file
    # We use a custom parser to preserve comments if possible, but standard ET is safer for simple scripts
    tree = ET.parse(WORLD_PATH)
    root = tree.getroot()
    world = root.find('world')

    # Models to randomize
    target_models = ['target_box', 'decoy_sphere', 'decoy_capsule', 'decoy_cylinder']
    
    # Existing heights (z-values) for each model to keep them on the floor
    # We can also extract them from the existing poses
    model_heights = {
        'target_box': 0.15,
        'decoy_sphere': 0.2,
        'decoy_capsule': 0.25,
        'decoy_cylinder': 0.15
    }

    # Generate unique positions for each model to avoid overlaps and occlusions
    placed_angles = []
    placed_positions = []
    
    # 50 degrees in radians
    MIN_ANGULAR_SEP = 50 * (math.pi / 180.0)
    
    for model_name in target_models:
        model_node = world.find(f".//model[@name='{model_name}']")
        if model_node is None:
            print(f"Warning: Model {model_name} not found in world.")
            continue

        # Generate a random position
        # Distance: 3m to 5m
        # Angle: 0 to 2pi
        valid_pos = False
        attempts = 0
        while not valid_pos and attempts < 100:
            attempts += 1
            r = random.uniform(3.0, 3.5)
            theta = random.uniform(0, 2 * math.pi)
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            
            # Check for overlap (min 1.0m distance)
            dist_ok = all(math.sqrt((x-px)**2 + (y-py)**2) > 1.0 for px, py in placed_positions)
            
            # Check for angular separation (min 50 degrees)
            # Use circular difference
            angle_ok = True
            for pa in placed_angles:
                diff = abs(theta - pa)
                if diff > math.pi:
                    diff = 2 * math.pi - diff
                if diff < MIN_ANGULAR_SEP:
                    angle_ok = False
                    break
            
            if dist_ok and angle_ok:
                valid_pos = True
                placed_positions.append((x, y))
                placed_angles.append(theta)

        z = model_heights.get(model_name, 0.15)
        
        # Update the pose element
        pose_node = model_node.find('pose')
        if pose_node is not None:
            # Format: x y z roll pitch yaw
            new_pose = f"{x:.3f} {y:.3f} {z:.3f} 0 0 0"
            pose_node.text = new_pose
            print(f"Moved {model_name} to: {x:.2f}, {y:.2f}")

    # Write back to file
    tree.write(WORLD_PATH, xml_declaration=True, encoding='utf-8')
    print("\nSuccessfully randomized shape positions in shapes.sdf")

if __name__ == "__main__":
    randomize_positions()
