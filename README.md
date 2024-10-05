This project focuses on developing a system for gesture recognition using data from six orientation sensors embedded in a hand glove.
The project consists of two parts: first, programming the sensors using Arduino to capture data at a minimum of 30 frames per second (FPS).
In the second part, data is collected from participants performing specific hand gestures.
The captured data is processed into JSON format, preprocessed, and split into training, validation, and test sets.
Using TensorFlow, a neural network is trained for binary classification to predict the gestures. Hyperparameters are optimized using Keras Tuner to enhance model accuracy.
