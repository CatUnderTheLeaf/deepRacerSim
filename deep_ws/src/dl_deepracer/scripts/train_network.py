#!/usr/bin/env python3
import tensorflow as tf
import keras
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, BatchNormalization, Dense, Flatten

def create_model(input_shape, pool_size):
    # a neural network from the paper 
    # "Real-Time End-to-End Self-Driving Car Navigation"
    # https://ijisae.org/index.php/IJISAE/article/view/2732
    # Yahya Ghufran Khidhir, Ameer Hussein Morad. (2023). 
    # Real-Time End-to-End Self-Driving Car Navigation. 
    # International Journal of Intelligent Systems and Applications in Engineering, 11(2s), 366–372.


    model = Sequential()
    # TODO
    # remove later when processing real data
    model.add(keras.Input(shape=input_shape))

    # BatchNorm2d-1
    model.add(BatchNormalization())
    

    # Conv2d-2
    model.add(Conv2D(24, (7, 7), padding='valid', strides=(1,1), activation = 'relu', name = 'Conv1'))

    # MaxPool2d-3
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="valid"))

    # Conv2d-4
    model.add(Conv2D(36, (5, 5), padding='valid', strides=(1,1), activation = 'relu', name = 'Conv2'))

    # MaxPool2d-5
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="valid"))

    # Conv2d-6
    model.add(Conv2D(64, (3, 3), padding='same', strides=(1,1), activation = 'relu', name = 'Conv3'))

    # MaxPool2d-7
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="valid"))

    # Conv2d-8
    model.add(Conv2D(64, (3, 3), padding='same', strides=(1,1), activation = 'relu', name = 'Conv4'))

    # MaxPool2d-9
    model.add(MaxPooling2D(strides=(2,2), pool_size=pool_size, padding="same"))

    # Conv2d-10 [-1, 64, 1, 10] 36,92
    model.add(Conv2D(64, (3, 3), padding='valid', strides=(1,1), activation = 'relu', name = 'Conv5'))

    model.add(Flatten())

    # Dense
    model.add(Dense(720))

    # Dense
    model.add(Dense(100))

    # Dense
    model.add(Dense(50))

    # Dense
    model.add(Dense(10))
    
    # Dense
    model.add(Dense(2))

    model.summary()
    return model


def main():

# images in dataset are 640*480
# TODO

    input_shape = (68,200,3)
    pool_size = (2,2)
    create_model(input_shape, pool_size)
    
    

if __name__ == '__main__':
    main()