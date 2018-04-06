import tensorflow as tf
import sys
import time
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #Load lable files
	print("TLClassifier Object Created")
        
        label=[line.rstrip() for line 
                in tf.gfile.GFile("retrained_labels_TF_130.txt")]

        #Load the graph
        with tf.gfile.FastGFile('retrained_graph_TF_130.pb', 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')

        print([n.name for n in tf.get_default_graph().as_graph_def().node])

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        tl_state = 3
	print("Image received from tl_detect")
	print(image)
        #image_data_enc = tf.image.encode_jpeg(image);
        image_data = tf.gfile.FastGFile(image, 'rb').read()
        print("Image Data Read Successfull")
        with tf.Session() as sess:
            softmax_tensor = sess.graph.get_tensor_by_name("final_result:0")
            # predictions = sess.run(softmax_tensor, image_data)
            predictions = sess.run(softmax_tensor, {'DecodeJpeg/contents:0': image_data})

            pred_label = predictions[0].argsort()[-len(predictions[0]):][::-1]

            print('predictions:')
            print(predictions[0][pred_label])
            print(pred_label)
            if(predictions[0][pred_label[0]] < 0.5):
                print("Classification is not good")
                tl_state = 3
            else:
                if(pred_label[0] == 1):
                    tl_state = 3
                    print("Light is Off")
                elif(pred_label[0] == 0):
                    print("Light is Green")
                    tl_state = 2
                elif(pred_label[0] == 3):
                    print("Light is Yellow")
                    tl_state = 1
                elif(pred_label[0] == 2):
                    print("Light is Red")
                    tl_state = 0

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
