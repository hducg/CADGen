

run commands in the following order, and change parameters after -r and -n appropriately
	python shape_sampler.py -r F:/wjcao/datasets/TestCAD -n 300 
	python generate_octree.py -o F:/wjcao/github/hducg/O-CNN/ocnn/octree/build/Release -r F:/wjcao/datasets/TestCAD -d 6 -n 1
	python generate_octree.py -o F:/wjcao/github/hducg/O-CNN/ocnn/octree/build/Release -r F:/wjcao/datasets/TestCAD -d 6 -n 12
	
modify path to lmdb in segmentation_6_cad.prototxt! and run commands for caffe training 
	python caffe_train_routine.py -c D:/Weijuan/caffe -r D:/Weijuan/dataset/cad -d 6

modify path to lmdb in segmentation_6_test.prototxt! run commands for caffe test	
	python caffe_test_routine.py -c D:/Weijuan/caffe -r D:/Weijuan/dataset/cad -b marche -n 6

run commands to display test results	
	python data_render.py -r F:/wjcao/datasets/TestCAD -b marche
