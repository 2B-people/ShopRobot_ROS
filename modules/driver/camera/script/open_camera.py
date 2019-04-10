import cv2
import os

class Camera(object):
    def __init__(self, url,delay_time):
        path = os.path.realpath(__file__)
        # self.path = path[:-36] + '/planning/distinguish_learn/shopping_detection/shopping_images'
        self.path = path[:-37] + '/planning/distinguish_learn/shopping_detection/shopping_images'
        self.url = url
        self.delay_time = delay_time

    def open_camera(self):
        cv2.namedWindow("camera", 1)
        capture = cv2.VideoCapture(self.url)

        while True:
            success, img = capture.read()
            cv2.imshow("camera", img)
            key = cv2.waitKey(10)
            if key == 27:
                print("esc break...")
                break

        capture.release()
        cv2.destroyWindow("camera")

    def get_photo(self, num):
        cv2.namedWindow("photo", 1)
        capture = cv2.VideoCapture(self.url)
        count = 0
        while True:
            success, image = capture.read()
            cv2.imshow("photo", image)
            count = count + 1
            if count == self.delay_time:
                name = str(num)
                # image = cv2.flip(image, 1)
                cv2.imwrite(self.path+'/image' + name + '.jpg', image)
                break
            cv2.waitKey(1)
        capture.release()
        cv2.destroyWindow("photo")


if __name__ == '__main__':
    url = "http://admin:admin@192.168.31.10:8081/"
    aa = Camera(url, 10)
    aa.get_photo(1)
