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
        #path_to_data       = os.path.abspath ("./light_classification/")
        path_to_data       = os.path.abspath (".")
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

        #self.detection_graph = tf.Graph()
	
	#with detection_graph.as_default():
	#	od_graph_def = tf.GraphDef()
	#	with tf.gfile.GFile (self.graph_file,'rb') as fid:
	#		serialized_graph = fid.read()
	#		od_graph_def.ParseFromString(serialized_graph)
	#		tf.import_graph_def(od_graph_def, name='')
        #         
	#	self.sess = tf.Session(graph=self.detection_graph)

	#self.label_map          = label_map_util.load_labelmap(self.label_name)
	#self.categories         = label_map_util.convert_label_map_to_categories(self.label_map,
        #										 max_num_classes=self.num_classes, 
        #									         use_display_name=True)

	#self.category_index     = label_map_util.create_category_index(self.categories)

	#self.image_tensor       = detection_graph.get_tensor_by_name('image_tensor:0')
    	# Each box represents a part of the image where a particular object was detected.

    	#self.detection_boxes    = detection_graph.get_tensor_by_name('detection_boxes:0')

    	# Each score represent how level of confidence for each of the objects.
    	# Score is shown on the result image, together with the class label.
    	#self.detection_scores   = detection_graph.get_tensor_by_name ('detection_scores:0')
    	#self.detection_classes  = detection_graph.get_tensor_by_name ('detection_classes:0')
    	#self.num_detections     = detection_graph.get_tensor_by_name ('num_detections:0')


	
   
    def read_tensor_from_image(self, image,input_height=299, input_width=299, input_mean=0, input_std=255):

       output_name = "normalized"

       #if file_name.endswith(".png"):
       #  image_reader = tf.image.decode_png(file_reader, channels = 3, name='png_reader')
       #elif file_name.endswith(".gif"):
       #   image_reader = tf.squeeze(tf.image.decode_gif(file_reader, name='gif_reader'))
       #elif file_name.endswith(".bmp"):
       #  image_reader = tf.image.decode_bmp(file_reader, name='bmp_reader')
       #else:
       image_reader = tf.image.decode_jpeg(image, channels = 3, name='jpeg_reader')


       float_caster  = tf.cast(image_reader, tf.float32)
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

	#image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

        #image_np_expanded             = np.expand_dims(image_np, axis=0)
        # Actual detection.
        #(boxes, scores, classes, num) = sess.run( [detection_boxes, detection_scores, detection_classes, num_detections], feed_dict={image_tensor: image_np_expanded})
	#predictions = self.isess.run(self.predictions,feed_dict={self.img_input: image})

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

