import json
import os
import sys

def modify_json_file(filepath):
    # Read the content of the JSON file
    with open(filepath, 'r') as file:
        data = json.load(file)
    
    # Modify the content
    for waypoint in data['waypoints']:
        waypoint['anchor']['x'] += 3.6576
        if waypoint['prevControl']:
            waypoint['prevControl']['x'] += 3.6576
        if waypoint['nextControl']:
            waypoint['nextControl']['x'] += 3.6576
        if waypoint['linkedName'] is not None:
            waypoint['linkedName'] += '_red'
    
    # Generate the new filename
    base, ext = os.path.splitext(filepath)
    new_filename = f"{base}_red{ext}"
    
    # Write the modified content to the new file, overwriting if it exists
    with open(new_filename, 'w') as file:
        json.dump(data, file, indent=2)
    
    print(f"Modified file saved as: {new_filename}")

def process_folder(folder):
    # Iterate through all files in the folder
    for filename in os.listdir(folder):
        if filename.endswith(".path") and not filename.endswith("_red.path"):
            filepath = os.path.join(folder, filename)
            modify_json_file(filepath)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python modify_json.py <folder>")
    else:
        folder_path = sys.argv[1]
        if os.path.isdir(folder_path):
            process_folder(folder_path)
        else:
            print(f"Error: {folder_path} is not a valid directory.")