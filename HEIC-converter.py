import os
from PIL import Image
from pillow_heif import register_heif_opener



root_dir = "label-space"

register_heif_opener()

file_to_remove = []

for root, dirs, files in os.walk(root_dir):

    for file in files:

        name = file.split('.')[0]
        extension = file.split('.')[-1]


        if extension == "HEIC":
            heic_path = os.path.join(root, file)
            jpg_path = os.path.join(root, f"{name}.jpg")


            image = Image.open(heic_path)
            image.convert('RGB').save(jpg_path, 'JPEG', quality=95)

            file_to_remove.append(heic_path)

for file in file_to_remove:
    os.remove(file)