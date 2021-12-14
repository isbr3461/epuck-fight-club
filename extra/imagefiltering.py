
import numpy as np
import cv2

color_ranges = []

def add_color_range_to_detect(lower_bound, upper_bound):

  global color_ranges
  color_ranges.append([lower_bound, upper_bound])

  
def check_if_color_in_range(bgr_tuple):
    global color_ranges
    for entry in color_ranges:
        lower, upper = entry[0], entry[1]
        in_range = True
        for i in range(len(bgr_tuple)):
          if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
            in_range = False
            break
        if in_range: return True
    return False


def do_color_filtering(img):
    img_height = img.shape[0]
    img_width = img.shape[1]

    mask = np.zeros([img_height, img_width])

    for x in range(0, img_width-1):
        for y in range(0, img_height-1):
          if(check_if_color_in_range(img[y][x])):
             mask[y][x] = 1

    return mask



def main():
    global img_height, img_width
    img = cv2.imread('./territorymap.png')
    add_color_range_to_detect([0,0,0], [230,230,230])
    img_mask = do_color_filtering(img)
    np.save("map.npy", img_mask)
    print("Map file saved")



if __name__ == '__main__':
  main()
    
