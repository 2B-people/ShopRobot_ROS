import cv2


class Camera(object):
    def __init__(self, url,path,delay_time):
        self.url = url
        self.path = path
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
            if count == delay_time:
                name = str(num)
                cv2.imwrite(path + name + '.jpg', image)
                break
            cv2.waitKey(1)
        capture.release()
        cv2.destroyWindow("photo")


if __name__ == '__main__':
    url = "http://admin:admin@192.168.31.102:8081/"
    aa = Camera(url)
    aa.get_photo(1)