import os
import json

"""
Takes label and image files from the label_space directory and converts them to the YOLO format.
Moving images to the appropriate directories and trashing unnecessary files.
Labels should be generated from labelme JSON files.
"""

root_dir = "label_space"
label_dir = "dataset/labels/val"
image_dir = "dataset/images/val"

images_to_move = []
files_to_trash = []

for root, dirs, files in os.walk(root_dir):


    for file in files:

        name = file.split('.')[0]
        extension = file.split('.')[-1]


        if extension == "jpg" or extension == "png":

            if name + ".json" in files:
                images_to_move.append(file)

        elif extension == "json":

            with open(label_dir + "/" + name + ".txt", 'w') as label_file:

                label_file.write("0 ")


                data = json.load(open(root_dir + "/" + file, 'r'))

                height = data["imageHeight"]
                width = data["imageWidth"]
                    

                for shape in data["shapes"]:
                    for point in shape["points"]:
                        label_file.write(f"{point[0]/width} {point[1]/height} ")

            files_to_trash.append(file)


for image in images_to_move:
    os.replace(root_dir + "/" + image, image_dir + "/" + image)

for file in files_to_trash:
    os.replace(root_dir + "/" + file, "trash/" + file)

            


