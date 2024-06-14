import os
import json
import sys

def update_pathnames_in_files(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".auto") and not filename.endswith("_red.auto"):
            input_filepath = os.path.join(directory, filename)
            output_filepath = os.path.join(directory, filename[:-5] + "_red.auto")
            with open(input_filepath, "r") as input_file:
                data = json.load(input_file)
            data["startingPose"]["position"]["x"] += 3.6576
                
            if "command" in data and "data" in data["command"] and "commands" in data["command"]["data"]:
                commands = data["command"]["data"]["commands"]
                for command in commands:
                    if "data" in command and "pathName" in command["data"]:
                        command["data"]["pathName"] += "_red"
                        
            with open(output_filepath, "w") as output_file:
                json.dump(data, output_file, indent=2)
                print(f"Created {output_filepath}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <directory>")
        sys.exit(1)
        
    directory = sys.argv[1]
    if not os.path.isdir(directory):
        print("Invalid directory path.")
        sys.exit(1)
        
    update_pathnames_in_files(directory)