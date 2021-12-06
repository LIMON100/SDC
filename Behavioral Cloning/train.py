import csv
import cv2
import numpy as np
import sklearn

from keras.layers.core.dropout import Dropout
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import Adam
from numpy.lib.function_base import angle

from sklearn.model_selection import train_test_split

sample = []

with open('data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    
    for line in reader:
        sample.append(line)

train_samples, validation_samples = train_test_split(sample, test_size=0.2)

augmented_images, augmneted_measurement = [], []

images = []
angles = []

def generator(samples, batch_size=32):
    num_samples = len(samples)

    while 1: 
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []

            for batch_sample in batch_samples:

                name = './IMG/'+batch_sample[0].split('/')[-1]
                center_image = cv2.imread(name)
                center_angle = float(batch_sample[3])

                images.append(center_image)
                angles.append(center_angle)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)

            yield sklearn.utils.shuffle(X_train, y_train)


for image, measurement in zip(images, angles):
    augmented_images.append(images)
    augmneted_measurement.append(augmneted_measurement)
    augmented_images.append(cv2.flip(image, 1))
    augmneted_measurement.append(measurement * - 1.0)


train_generator = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)

ch, row, col = 3, 80, 320

model = Sequential()

model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape = (160, 320, 3)))

model.add(Cropping2D(cropping=((70, 25),(0,0)))) 

model.add(Convolution2D(24, 5, 5, subsample = (2, 2), activation = 'relu'))
model.add(Convolution2D(36, 5, 5, subsample = (2, 2), activation = 'relu'))
model.add(Convolution2D(48, 5, 5, subsample = (2, 2), activation = 'relu'))
model.add(Convolution2D(64, 3 ,3 , activation = 'relu'))
model.add(Convolution2D(64, 3, 3, activation = 'relu'))

model.add(Flatten())
model.add(Dropout(0.5))
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dropout(0.5))
model.add(Dense(1))

model.compile(loss = 'mse', optimizer = 'adam')
model.fit(train_generator, samples_per_epoch = len(train_samples), validation_data=validation_generator, nb_val_samples=len(validation_samples), nb_epoch=3)

model.save("train_model.h5")