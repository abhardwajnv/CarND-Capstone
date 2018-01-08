
import rospy
import rospkg
import os
import time
import sys
import numpy as np
import tensorflow as tf
from styx_msgs.msg import TrafficLight
from collections import defaultdict
from utils import label_map_util
from utils import visualization_utils as vis_util
from io import StringIO


class TLClassifier(object):
    def __init__(self, real = False):
        # set default value for no detection
        n_classes = 4
        self.traffic_light = TrafficLight.UNKNOWN
        
        model_path = os.path.dirname(os.path.realpath(__file__))
        label_file = model_path + '/label_map.pbtxt'
        if real:
            model = model_path + '/data/real_model/frozen_inference_graph.pb'
        else:
		    model = model_path + '/data/sim_model/frozen_inference_graph.pb'    
  

        label_map = label_map_util.load_labelmap(label_file)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_n_classes=n_classes,use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.image_np_deep = None
        self.detection_graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

        self.scores = self.detection_graph.get_tensor_by_name('scores:0')
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.classes = self.detection_graph.get_tensor_by_name('classes:0')
        self.bboxes = self.detection_graph.get_tensor_by_name('bboxes:0')
        self.detections = self.detection_graph.get_tensor_by_name('detections:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.traffic_light = TrafficLight.UNKNOWN
        image_expanded = np.expand_dims(image, axis=0)
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.bboxes, self.scores,
                 self.classes, self.detections],
                feed_dict={self.image_tensor: image_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_thresh = 0.65
        for box in range(boxes.shape[0]):
            if scores is None or scores[box] > min_thresh:
                tl_color = self.category_index[classes[box]]['name']
                if tl_color == 'Red':
                    self.traffic_light = TrafficLight.RED
                elif tl_color == 'Yellow':
                    self.traffic_light = TrafficLight.YELLOW
                elif tl_color == 'Green':
                    self.traffic_light = TrafficLight.GREEN

                self.image_np_deep = image

        if (self.traffic_light == TrafficLight.UNKNOWN):
            tl_color = 'UNKNOWN'
