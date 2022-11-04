import numpy as np


def gaussian_noise(img):
    gauss = np.random.normal(0, 1, img.size)
    gauss = gauss.reshape(
        img.shape[0], img.shape[1], img.shape[2]).astype('uint8')
    noise = img + img * gauss
    return noise
