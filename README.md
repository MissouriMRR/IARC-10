# New Yolo Trainer

## Training the model

### Creating a model

If you want to make a new model instead of training the currently built model you can use the **build-model.py**.
You can configure the model by changing the **model_to_build**. 
The **model_to_build** represents a pretrained yolo model that we can further train to achieve or goals.
The naming conventions are as follows.

> yolo26[LETTER REPRESENTING SIZE][add "-obb" for a oriented rectange model].pt

The potentail sizes of the model are: 
 * **n** for nano
 * **s** for small
 * **m** for medium
 * **l** for large
 * **x** for extra large

As the models become larger they become more accurate but slower and more energy intensive.

You can also configure **epochs**. These represent the number of cycles of the training images that the model will initially train on.

### Training the built model

The **train_model.py** file can be used to train models. To do so you must set the **model_path** to the corresponding model and set the **epochs** to the number of times you want to train the model on each image.

### Evaluating a model

The easiest way to evaluate if the model is functioning properly is to got to its training directory.
This should be at runs/[obb or detect]/train.

There you can find are a variety of files that give insight into the functioning of the model.
One such file is the **val_batch0_pred.jpg** image.
This image shows the bounding boxes that are given to the mines by the model. 
Another way to evaluate the model is through the **confusion_matrix.png**.
This displays the relationship between the models predicted locations of mines and the actual.

## Creating the dataset

### Uploading Images

The **label_space** directory is where you should be uploading the images. They must be _jpg_ or _png_ files.

### Creating Labels

Once you have you images uploaded, you can create the labels. The labels should be created using the free [**_LabelMe_**](https://labelme.io/download) software. Labels should be a rectangle around the mines. The labels should complete fit the mine inside with little with minimum space between. For this project all the labels should be named "_0_" since we only have on object type. If you are training an oriented rectangle model use oriented rectangles other with use normal rectangles.

### Formating

After the labels are created you can run the **label-converter.py** file to move them into the correct places. You can edit the destinations of the images and labels by changing the **image_dir** and **label_dir** files.

## Data Organization

The way that data is organized is in correspondence with the [Yolo Documentation](https://docs.ultralytics.com/usage/python) which I highly recommend you read through. 
The main things you need to understand is what **data.yaml** does and how the **datasets** directory works

Honestly I might add info on this later but for now I am to lazy. If you need to change something just read  first or just figure it out. If something goes wrong we always have github :\).