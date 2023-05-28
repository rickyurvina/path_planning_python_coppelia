import numpy as np
import cv2
import matplotlib.pyplot as plt
import pickle

# Cargar las variables guardadas en el archivo .pickle
with open('../files/routes1.pickle', 'rb') as f:
    data = pickle.load(f)
    routes = data['routes']


