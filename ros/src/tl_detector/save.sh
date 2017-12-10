export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
python object_detection/export_inference_graph.py --input_type image_tensor --pipeline_config_path model/faster_rcnn_resnet101.config --trained_checkpoint_prefix ../../train/model.ckpt-20000 --output_directory ../../
