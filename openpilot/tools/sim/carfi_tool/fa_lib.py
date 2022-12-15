import numpy as np
import cv2
from PIL import Image, ImageEnhance
import util


def gaussian(img, fault):
    seed = util.get_config_val(fault['seed'])
    fmin = util.get_config_val(fault['min'])
    fmax = util.get_config_val(fault['max'])
    np.random.seed(seed)
    gauss = np.random.normal(fmin, fmax, img.size)
    gauss = gauss.reshape(
        img.shape[0], img.shape[1], img.shape[2]).astype('uint8')
    return img + img * gauss


def bac(img, fault):
    factor = util.get_config_val(fault['factor'])
    image = Image.fromarray(img)
    enhancer = ImageEnhance.Brightness(image)
    return np.asarray(enhancer.enhance(factor))


def poisson(img, fault):
    lam = util.get_config_val(fault['lam'])
    return img + np.random.poisson(lam)


def speckle(img, fault):
    return img + img * np.random.randn(img.shape).astype('uint8')


def blur(img, fault):
    kh = util.get_config_val(fault['kernel hight'])
    kw = util.get_config_val(fault['kernel width'])
    return cv2.blur(img, (kw, kh))


def dead_pixel(img, fault):
    h = util.get_config_val(fault['hight'])
    w = util.get_config_val(fault['width'])
    img[h, w] = (0, 0, 0)
    return img


def sharpness(img, fault):
    factor = util.get_config_val(fault['factor'])
    enhancer = ImageEnhance.Sharpness(img)
    return enhancer.enhance(factor)
