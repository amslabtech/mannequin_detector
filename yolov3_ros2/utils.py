import sys
import os
from PIL import Image, ImageFont, ImageDraw
import numpy as np
import matplotlib.pyplot as plt
import cv2
import colorsys

def take_contours(objects):
    for obj in objects:
        if 'mask' in obj:
            mask = obj['mask']
            # Take contours
            contours, hierarchy = cv2.findContours(
                mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            polygon = []
            for cnt in contours:
                ps=[]
                if (len(cnt) <= 2):
                    continue
                for point in cnt:
                    x, y = point[0]
                    ps.append(int(x))
                    ps.append(int(y))
                polygon.append(ps)

            all_points_x = [k for contour in contours for i in contour.tolist() if len(i) >= 1 for j in i for k in j[0::2]],
            all_points_y = [k for contour in contours for i in contour.tolist() if len(i) >= 1 for j in i for k in j[1::2]],
            obj.update({
                "contours": contours,
                "hierarchy": hierarchy,
                "polygon": polygon,
                "all_points_x": all_points_x,
                "all_points_y": all_points_y
            })

    return objects


def make_r_image(image, objects, colors, alpha=0.3):
    image = np.array(image)
    image = np.array(image[..., ::-1], dtype=np.float32)
    polygons = []
    for obj in objects:
        x1, y1, x2, y2 = obj['bbox']
        score = obj['score']
        class_name = obj['class_name']
        class_id = obj['class_id']
        color = colors[class_id]
        contours = False
        if 'mask' in obj:
            mask = obj['mask']
            image[mask] = (image[mask] * (1.0 - alpha) + np.array(color) * alpha)
            contours = obj['contours']
            cv2.drawContours(image, contours, -1, color, 2)

        print(x1)
        print(y1)
        print(x2)
        print(y2)
        print(color)

        # Draw rectangular bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        # Label
        cv2.putText(image, "%i %s %.2f" % (class_id, class_name, score),
                    (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (255, 255, 255))

    image = image[..., ::-1]  # Convert back to PIL
    return Image.fromarray(image.astype(np.uint8))

def deprocess_feature(x):
    """Deprocess feature x for visualization

    Args:
    - x: 3 dimensional vector (w x h x depth)
    """

    x = x - np.mean(x, axis=-1, keepdims=True)
    x /= np.std(x, axis=-1, keepdims=True) + 1e-5
    return np.clip(x * 0.1 + 0.5, 0.0, 1.0)

def visualize_and_save(feature, save_path):
    """Visualize feature values"""

    feature = deprocess_feature(feature)
    fig, axes = plt.subplots(6, 10, sharex=True, sharey=True)
    for index, axis in enumerate(axes.flat):
        plotted_im = axis.imshow(feature[..., index])
        axis.set_axis_off()
    fig.colorbar(plotted_im, ax=axes.ravel().tolist())
    plt.savefig(save_path)

class NotFoundError(Exception):
    pass

def get_unused_dir_num(pdir, pref=None):
    os.makedirs(pdir, exist_ok=True)
    dir_list = os.listdir(pdir)
    for i in range(1000):
        search_dir_name = "" if pref is None else (
            pref + "_" ) + '%03d' % i
        if search_dir_name not in dir_list:
            return os.path.join(pdir, search_dir_name)
    raise NotFoundError('Error')

def generate_colors(class_num):
    hsv_tuples = [(x / class_num, 1., 1.)
                    for x in range(class_num)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(
        map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
            colors))
    np.random.seed(10101)  # Fixed seed for consistent colors across runs.
    np.random.shuffle(colors)  # Shuffle colors to decorrelate adjacent classes.
    np.random.seed(None)  # Reset seed to default.
    return colors