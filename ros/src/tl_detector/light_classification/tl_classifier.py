import tensorflow as tf
import sys
import time
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #Load lable files
        
        label=[line.rstrip() for line 
                   in tf.gfile.GFile("retrained_labels.txt")]

        #Load the graph
        with tf.gfile.FastGFile("retrained_graph.pb", 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')
          
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        tl_state = TrafficLight.UNKNOWN
        image_data = tf.gfile.FastGFile(image, 'rb').read()
        with tf.Session() as sess:
            softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
            predictions = sess.run(softmax_tensor,{'DecodeJpeg/contents:0': image_data})
		      	
            pred_label = predictions[0].argsort()[-len(predictions[0]):][::-1]
            print(predictions[0][pred_label])
            print(pred_label)
            if(predictions[0][pred_label[0]] < 0.5):
                print("Classification is not good")
                tl_state = TrafficLight.UNKNOWN
            else:
                if(pred_label[0] == 0):
                    tl_state = TrafficLight.UNKNOWN
                    print("Light is Off")
                elif(pred_label[0] == 1):
                    print("Light is Green")
                    tl_state = TrafficLight.GREEN
                elif(pred_label[0] == 2):
                    print("Light is Yellow")
                    tl_state = TrafficLight.YELLOW
                elif(pred_label[0] == 3):
                    print("Light is Red")
                    tl_state = TrafficLight.RED

        return tl_state

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
