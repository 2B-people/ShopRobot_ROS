# coding=utf-8
# import os
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
import numpy as np
import tensorflow as tf
from PIL import Image
from utils import ops as utils_ops
import time
import os
import os


class Object_D(object):
    def __init__(self):
        path = os.path.realpath(__file__)
        # print path
        # bug 因为py生成了pyc文件,在实际调用时为pyc文件,影响到了字符串的索引
        # path_model = path[:-17] + '/model'
        # self.path_image = path[:-17] + '/shopping_images'
        path_model = path[:-18] + '/model'
        self.path_image = path[:-18] + '/shopping_images'
        self.MODEL_NAME = path_model
        self.PATH_TO_FROZEN_GRAPH = self.MODEL_NAME + '/frozen_inference_graph.pb'
        
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

    def run_inference_for_single_image(self, image, graph):
        with graph.as_default():
            with tf.Session() as sess:
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                    'num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes', 'detection_masks'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                            tensor_name)
                if 'detection_masks' in tensor_dict:
                    # The following processing is only for single image
                    detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                    detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                    # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                    real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                    detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                    detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                    detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                        detection_masks, detection_boxes, image.shape[0], image.shape[1])
                    detection_masks_reframed = tf.cast(
                        tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                    # Follow the convention by adding back the batch dimension
                    tensor_dict['detection_masks'] = tf.expand_dims(
                        detection_masks_reframed, 0)
                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

                # Run inference
                output_dict = sess.run(tensor_dict,
                                       feed_dict={image_tensor: np.expand_dims(image, 0)})

                # all outputs are float32 numpy arrays, so convert types as appropriate
                output_dict['num_detections'] = int(output_dict['num_detections'][0])
                output_dict['detection_classes'] = output_dict[
                    'detection_classes'][0].astype(np.uint8)
                output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                output_dict['detection_scores'] = output_dict['detection_scores'][0]
                if 'detection_masks' in output_dict:
                    output_dict['detection_masks'] = output_dict['detection_masks'][0]
        return output_dict

    def get_qualified_num(scores):
        num = 0
        for i in range(3):
            if(scores[i] >= 0.5):
                num += 1
        return num

    def swap_box(self, boxes, classes, scores):
        result = {}
        boxes = boxes.tolist()
        classes = classes.tolist()
        scores = scores.tolist()
        num = self.get_qualified_num(scores)
        if(num == 3 or num == 0):
            num_max = boxes.index(max(boxes))
            if num_max != 2:
                boxes[num_max], boxes[2] = boxes[2], boxes[num_max]
                classes[num_max], classes[2] = classes[2], classes[num_max]
                scores[num_max], scores[2] = scores[2], scores[num_max]

            num_min = boxes.index(min(boxes))
            if num_min != 0:
                boxes[num_min], boxes[0] = boxes[0], boxes[num_min]
                classes[num_min], classes[0] = classes[0], classes[num_min]
                scores[num_min], scores[0] = scores[0], scores[num_min]


            result['classes'] = classes
            result['scores'] = scores

            return result
        elif(num == 2):
            flag[] = [True, True, True]
            new_classes[] = [-1, -1, -1]
            new_scores[] = [-1, -1, -1]
            for i in range(3):
                if(scores[i] >= 0.5):
                    flag[i] = False
                    if(boxes[i] <= 0.2):
                        new_classes[0] = classes[i]
                        new_scores[0] = scores[i]
                    if(0.3<=boxes[i]<=0.5):
                        new_classes[1] = classes[i]
                        new_scores[1] = scores[i]
                    if(0.6<=boxes[i]):
                        new_classes[2] = classes[i]
                        new_scores[2] = scores[i]
            for i in range(3):
                if(flag[i]):
                    for j in range(3):
                        if(new_classes[j] != -1):
                            new_classes[j] = classes[i]
                            new_scores[j] = scores[i]

            result['classes'] = new_classes
            result['scores'] = new_scores

            return result
        elif(num == 1):
            location = -1
            flag_local = []
            flag[] = [True, True, True]
            new_classes[] = [-1, -1, -1]
            new_scores[] = [-1, -1, -1]
            for i in range(3):
                flag[i] = False 
                if(scores[i]>=0.5):
                    if(boxes[i] <= 0.2):
                        new_classes[0] = classes[i]
                        new_scores[0] = scores[i]
                        location = 0
                    if(0.3<=boxes[i]<=0.5):
                        new_classes[1] = classes[i]
                        new_scores[1] = scores[i]
                        location = 1
                    if(0.6<=boxes[i]):
                        new_classes[2] = classes[i]
                        new_scores[2] = scores[i]
                        location = 2
            for i in range(3):
                if(flag[i]):
                    flag_local.append(i)
            if(location == 0):
                if(boxes[flag_local[1]] >= boxes[flag_local[0]]):
                    new_classes[1] = classes[flag_local[0]]
                    new_scores[1] = scores[flag_local[0]]
                    new_classes[2] = classes[flag_local[1]]
                    new_scores[2] = scores[flag_local[1]]
                else:
                    new_classes[1] = classes[flag_local[1]]
                    new_scores[1] = scores[flag_local[1]]
                    new_classes[2] = classes[flag_local[0]]
                    new_scores[2] = scores[flag_local[0]]
            elif(location == 1):
                if(boxes[flag_local[1]] >= boxes[flag_local[0]]):
                    new_classes[0] = classes[flag_local[0]]
                    new_scores[0] = scores[flag_local[0]]
                    new_classes[2] = classes[flag_local[1]]
                    new_scores[2] = scores[flag_local[1]]
                else:
                    new_classes[0] = classes[flag_local[1]]
                    new_scores[0] = scores[flag_local[1]]
                    new_classes[2] = classes[flag_local[0]]
                    new_scores[2] = scores[flag_local[0]]
            elif(location == 2):
                if(boxes[flag_local[1]] >= boxes[flag_local[0]]):
                    new_classes[0] = classes[flag_local[0]]
                    new_scores[0] = scores[flag_local[0]]
                    new_classes[1] = classes[flag_local[1]]
                    new_scores[1] = scores[flag_local[1]]
                else:
                    new_classes[0] = classes[flag_local[1]]
                    new_scores[0] = scores[flag_local[1]]
                    new_classes[1] = classes[flag_local[0]]
                    new_scores[1] = scores[flag_local[0]]
            
            result['classes'] = new_classes
            result['scores'] = new_scores

            return result




    def run_image(self, num):
        name = str(num)
        boxes = []
        image = Image.open(self.path_image + '/image' + name + '.jpg')
        # image = Image.open('shopping_images/image1.jpg')
        image_np = self.load_image_into_numpy_array(image)
        output_dict = self.run_inference_for_single_image(image_np, self.detection_graph)
        for i in range(3):
            boxes.append(output_dict['detection_boxes'][i][1])
        classes = output_dict['detection_classes'][0:3]
        scores = output_dict['detection_scores'][0:3]
        result = self.swap_box(boxes, classes, scores)
        return result


if __name__ == '__main__':
    a = Object_D()
    # image1 = Image.open('test_images/image1.jpg')
    box = a.run_image(1)
    # print box[0]
    # print box[1]
    # print box['classes']
    print box


