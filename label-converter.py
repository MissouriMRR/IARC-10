import os
import json

"""
Takes label and image files from the label_space directory and converts them to the YOLO format.
Moving images to the appropriate directories and trashing unnecessary files.
Labels should be generated from labelme JSON files.
"""

label_space_dir = "label-space"

data_dir = "datasets"
label_dir = f"{data_dir}/labels/train"
image_dir = f"{data_dir}/images/train"

images_to_move = []
files_to_trash = []

for root, dirs, files in os.walk(label_space_dir):


    for file in files:

        print(file)

        name = file.split('.')[0]
        extension = file.split('.')[-1]


        if extension == "jpg" or extension == "png" or extension == "jpeg":

            if name + ".json" in files:
                images_to_move.append(file)

        elif extension == "json":

            with open(label_dir + "/" + name + ".txt", 'w') as label_file:

                


                data = json.load(open(label_space_dir + "/" + file, 'r'))

                height = data["imageHeight"]
                width = data["imageWidth"]
                    

                for shape in data["shapes"]:
                    label_file.write("0 ")
                    for point in shape["points"]:
                        label_file.write(f"{point[0]/width} {point[1]/height} ")
                    label_file.write("\n")

            files_to_trash.append(file)


for image in images_to_move:
    os.replace(label_space_dir + "/" + image, image_dir + "/" + image)

for file in files_to_trash:
    os.replace(label_space_dir + "/" + file, "trash/" + file)

            


