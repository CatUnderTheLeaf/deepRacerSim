#!/usr/bin/env python3
import keras
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, BatchNormalization, Dense, Flatten, Dropout, Resizing
from keras.utils import load_img, img_to_array

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

import math
import numpy as np
import glob
import os
import csv

def create_model(input_shape, pool_size):
    # my reconstruction of a neural network from the paper 
    # "Real-Time End-to-End Self-Driving Car Navigation"
    # https://ijisae.org/index.php/IJISAE/article/view/2732
    # Yahya Ghufran Khidhir, Ameer Hussein Morad. (2023). 
    # Real-Time End-to-End Self-Driving Car Navigation. 
    # International Journal of Intelligent Systems and Applications in Engineering, 11(2s), 366–372.


    model = Sequential()
    # to test only model without real data
    # model.add(keras.Input(shape=input_shape))

    # resize images to minimize number of features
    model.add(Resizing(150,200))

    # BatchNorm2d-1
    model.add(BatchNormalization(input_shape=input_shape))
    

    # Conv2d-2
    model.add(Conv2D(24, (7, 7), strides=(1,1), activation = 'relu'))

    # MaxPool2d-3
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="valid"))

    # Conv2d-4
    model.add(Conv2D(36, (5, 5), strides=(1,1), activation = 'relu'))

    # MaxPool2d-5
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="valid"))

    # Conv2d-6
    model.add(Conv2D(64, (5, 5), strides=(1,1), activation = 'relu'))

    # MaxPool2d-7
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="valid"))

    # Conv2d-8
    model.add(Conv2D(64, (3, 3), strides=(1,1), activation = 'relu'))

    # MaxPool2d-9
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size))

    # Conv2d-10
    model.add(Conv2D(64, (3, 3), strides=(1,1), activation = 'relu'))

    model.add(Flatten())

    # Dense
    model.add(Dense(720))

    # Dense
    model.add(Dense(100))
    model.add(Dropout(0.2))

    # Dense
    model.add(Dense(50))
    model.add(Dropout(0.2))

    # Dense
    model.add(Dense(10))
    
    # Dense
    model.add(Dense(2))

    # model.summary()
    return model

class ImageGeneratorSequence(keras.utils.Sequence):
    ''' 
    a class for loading images from lines of a .csv in batches
    every line in .csv file looks like:
    line[0] = path to the image
    line[1] = speed
    line[2] = steering angle
    
    '''
    def __init__(self, lines, batch_size):
        self.lines = lines
        self.batch_size = batch_size

    def __len__(self):
        return math.ceil(len(self.lines) / self.batch_size)

    def __getitem__(self, idx):
        low = idx * self.batch_size
        # Cap upper bound at array length; the last batch may be smaller
        # if the total number of items is not a multiple of batch size.
        high = min(low + self.batch_size, len(self.lines))
        batch_data = self.lines[low:high]
        shuffle(batch_data)        
        
        # TODO
        # add flipped images and angles to augment the data set
          
        return np.array([
            img_to_array(load_img(line[0]))
               for line in batch_data]), np.array([(float(line[1]), float(line[2])) for line in batch_data])


def main():

    input_shape = (480, 640,3)
    pool_size = (3,3)
    batch_size = 64
    epochs = 70

    
    # read lines from csv-file
    lines = []
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data', '*.csv')
    print(glob.glob(path))
    for file in glob.glob(path):
        with open(file) as csvfile:
            reader = csv.reader(csvfile)
            next(reader, None)
            for line in reader:
                lines.append(line)
    
    # split images in train and validation sets
    train_set, validation_set = train_test_split(lines, test_size=0.2)
   
    train_seq = ImageGeneratorSequence(train_set, batch_size)
    val_seq = ImageGeneratorSequence(validation_set, batch_size)
    
    model = create_model(input_shape, pool_size)
    model.compile(optimizer=keras.optimizers.Adam(learning_rate=5e-4), loss='mean_squared_error')
    # TODO
    # add early stopping if no changes occurred
    # in validation loss within 10 epochs
    model.fit(train_seq, batch_size=batch_size, epochs=epochs, validation_data=val_seq)

    model.trainable = False
    
    # Save model architecture and weights
    model_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'model', 'deepRacer_model.keras')
    model.save(model_path)
    
    

if __name__ == '__main__':
    main()


# conda activate
# CUDNN_PATH=$(dirname $(python3 -c "import nvidia.cudnn;print(nvidia.cudnn.__file__)"))
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDNN_PATH/lib:$CONDA_PREFIX/lib/