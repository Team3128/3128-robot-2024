import json
import sys
import os

def modify_json_file(filename):
    # Read the content of the JSON file
    with open(filename, 'r') as file:
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
    base, ext = os.path.splitext(filename)
    new_filename = f"{base}_red{ext}"
    
    # Write the modified content to the new file
    with open(new_filename, 'w') as file:
        json.dump(data, file, indent=2)
    
    print(f"Modified file saved as: {new_filename}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python modify_json.py <filename>")
    else:
        modify_json_file(sys.argv[1])