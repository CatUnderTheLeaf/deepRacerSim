#!/usr/bin/env python3

from keras.models import load_model

import os
import cv2
import numpy as np




def main():

    model_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'model', 'deepRacer_model.keras')
    model = load_model(model_path)
    img_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images', '1692969336.898314.jpg')
    test_img = cv2.imread(img_path)
    test_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB)
    
    # model.summary()

    prediction = model.predict(np.array([test_img]))
    print(prediction)
    

if __name__ == '__main__':
    main()