import tensorflow as tf
import sys
import time
from styx_msgs.msg import TrafficLight
from shutil import copyfile
import datetime
import os

class TLClassifier(object):
    def __init__(self):
        #Load lable files

        label=[line.rstrip() for line
                in tf.gfile.GFile("retrained_labels_TF_130.txt")]

        self.graph = tf.Graph()
        with self.graph.as_default():
            #Load the graph
            with tf.gfile.FastGFile('retrained_graph_TF_130.pb', 'rb') as f:
                graph_def = tf.GraphDef()
                graph_def.ParseFromString(f.read())
                _ = tf.import_graph_def(graph_def, name='')

        # print([n.name for n in tf.get_default_graph().as_graph_def().node])
        self.sess = tf.Session(graph=self.graph)
        self.img_num = 0
        self.time_dir = datetime.datetime.now().strftime('%04Y%02m%02d_%02H%02M%02S')
        self.pprefix = '/tmp/' + self.time_dir
        self.prefix = self.pprefix + '/lights'
        if False:  # debug things
            os.mkdir(self.pprefix)
            os.mkdir(self.prefix)
            os.mkdir(self.prefix + '/green')
            os.mkdir(self.prefix + '/red')
            os.mkdir(self.prefix + '/yellow')
            os.mkdir(self.prefix + '/nogood')
            os.mkdir(self.prefix + '/off')
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color based on the trained model labels of file 			 retrained_labels_TF_130.txt
        """
        tl_state = 0  ## Initialize with Red light

        image_data = tf.gfile.FastGFile(image, 'rb').read()
        softmax_tensor = self.sess.graph.get_tensor_by_name("final_result:0")
        predictions = self.sess.run(softmax_tensor, {'DecodeJpeg/contents:0': image_data})
        dst = '/tmp/lights/nogood/%08d.jpg'

        pred_label = predictions[0].argsort()[-len(predictions[0]):][::-1]

        if predictions[0][pred_label[0]] < 0.45:  # If prediction probability is less than 0.5 then classification is not good enough
            print("Classification is not good")
            tl_state = 0  # if classification is not good. consider it to be red (safe option)
            dst = self.prefix + '/nogood/%08d.jpg'
        else:
            if pred_label[0] == 100:    # 1: not used anymore
                tl_state = 3  # Treat an off as random state. This is not handled in the tl_detector.
                print("Light is Off: %06d" % self.img_num)
                dst = self.prefix + '/off/%08d.jpg'
            elif pred_label[0] == 0:
                print("Light is Green: %06d" % self.img_num)
                tl_state = 2
                dst = self.prefix + '/green/%08d.jpg'
            elif pred_label[0] == 2:  # 3:
                print("Light is Yellow: %06d" % self.img_num)
                tl_state = 1
                dst = self.prefix + '/yellow/%08d.jpg'
            elif pred_label[0] == 1:   # 2:
                print("Light is Red: %06d" % self.img_num)
                tl_state = 0
                dst = self.prefix + '/red/%08d.jpg'
        if False:  # debug things
            copyfile(image, dst % self.img_num)
        self.img_num += 1
        return tl_state

## Below is code is for only testing of this class.
#x=time.time()
#testobject = TLClassifier()
#print(time.time()-x)
#y=time.time()
#testobject.get_classification('testImage.jpg')
#print(time.time()-y)
#z=time.time()
#testobject.get_classification('0r1.jpeg')
#print(time.time()-z)
#testobject.get_classification('0r3.jpeg')
