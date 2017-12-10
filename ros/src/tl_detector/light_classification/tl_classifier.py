#from styx_msgs.msg import TrafficLight
import glob
import tensorflow as tf
import numpy as np
import os
import sys

class TLClassifier(object):

    def load_graph(self,model_file):
        graph     = tf.Graph()
        graph_def = tf.GraphDef()

        with open(model_file, "rb") as f:
             graph_def.ParseFromString(f.read())
        with graph.as_default():
             tf.import_graph_def(graph_def)

        return graph
	
    def __init__(self):
        
        model_name         = "/mobilenet_1.0_224.pb"
        path_to_data       = os.path.abspath ("./light_classification/")
        #path_to_data       = os.path.abspath (".")
        label_name         =  "/labels.pbtxt"
        self.num_classs    = 4

        graph_file    = path_to_data +  model_name
        label_file    = path_to_data + label_name


        self.Model         = self.load_graph  (graph_file)
        self.Labels        = self.load_labels (label_file)

        self.height       = 224
        self.width         = 224
        self.mean          = 128
        self.std           = 128
        self.input_layer   = "input"
        self.output_layer  = "final_result"



	
   
    def read_tensor_from_image(self, image,input_height=299, input_width=299, input_mean=0, input_std=255):

       output_name = "normalized"


       float_caster  = tf.cast(image, tf.float32)
       dims_expander = tf.expand_dims(float_caster, 0);
       resized       = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
       normalized    = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
       sess          = tf.Session()
       result        = sess.run(normalized)

       return result


    def load_labels(self,label_file):

        label = []
        proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()

        for l in proto_as_ascii_lines:
             label.append(l.rstrip())

        return label
	

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color predictio


        t = self.read_tensor_from_image(image, input_height=self.height, input_width =self.width, input_mean  =self.mean,input_std   =self.std)

        input_name       = "import/" + self.input_layer
        output_name      = "import/" + self.output_layer
        input_operation  = self.Model.get_operation_by_name(input_name)
        output_operation = self.Model.get_operation_by_name(output_name)

        with tf.Session(graph=self.Model) as sess:
             results = sess.run(output_operation.outputs[0],{input_operation.outputs[0]:t})

        results = np.squeeze(results)
	
        top_k = results.argsort()[-5:][::-1]
      
        #print ("here",results," ",",",top_k)

        return self.Labels[top_k[0]]


if __name__ == "__main__":

       tlc = TLClassifier()
	
       for fname in glob.glob("/home/dcml/project4/retrain/test/*.jpg"):


        #img       = Image.open(fname)
        #img       = img.resize ((width,height))
           input_name  = "file_reader"
           img         = tf.read_file(fname,input_name)

           print (fname, tlc.get_classification(img))	
      
       print (tlc.Labels)

