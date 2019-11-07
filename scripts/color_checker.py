import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse

class ColorChecker():
    def __init__(self, path, check, lower, upper):
        self.image = cv2.imread(path)
        self.check = check
        self.lower = np.array(self.str_to_list(lower))
        self.upper = np.array(self.str_to_list(upper))
        self.start()
    
    def str_to_list(self, s):
        s_nums = s.split(',')
        nums = []
        for s_num in s_nums:
            nums.append(int(s_num))
        return nums
    
    def start(self):
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        if self.check:
            im_list = np.asarray(hsv_image)
            plt.imshow(im_list)
            plt.show()
            return
        print(self.lower)
        print(self.upper)
        binarized_image = cv2.inRange(hsv_image, self.lower, self.upper)
        masked_image = cv2.bitwise_and(self.image, self.image, mask=binarized_image)
        cv2.imshow("original", self.image)
        cv2.imshow("masked", masked_image)
        cv2.waitKey(0)



def main():
    parser = argparse.ArgumentParser()
    
    parser.add_argument('-p', '--path', type=str, help='path to image', default='/home/amsl/Pictures/image.png')
    parser.add_argument('-c', '--check', action='store_true', help='check hsv')
    parser.add_argument('-l', '--lower', type=str, help='input lower hsv. format is "h,s,v"', default='0,0,0')
    parser.add_argument('-u', '--upper', type=str, help='input upper hsv. format is "h,s,v"', default='0,0,0')

    args = parser.parse_args()
    
    color_checker = ColorChecker(args.path, args.check, args.lower, args.upper)




if __name__ == '__main__':
    main()