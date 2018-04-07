#importing some useful packages
import numpy as np
import cv2
from keras.models import load_model

LABELS = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
WANTED_LABELS = ['traffic light']
WANTED_IDXS=[]
for i in range (len(WANTED_LABELS)):
		WANTED_IDXS.append(LABELS.index(WANTED_LABELS[i]))


IMAGE_H, IMAGE_W = 416, 416
GRID_H,  GRID_W  = 13 , 13
BOX						  = 5
CLASS						= len(LABELS)
CLASS_WEIGHTS		= np.ones(CLASS, dtype='float32')


#ANCHORS = [18.3274,21.6763,  59.9827,66.001,  106.83,175.179,  252.25,112.889,  312.657,293.385]
#ANCHORS=np.array(ANCHORS)/25
ANCHORS = [0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828]
ANCHORS=np.array(ANCHORS)*1.4
OBJ_THRESHOLD		= 0.18
NMS_THRESHOLD		= 0

NO_OBJECT_SCALE  = 1.0
OBJECT_SCALE		 = 5.0
COORD_SCALE		  = 1.0
CLASS_SCALE		  = 1.0

BATCH_SIZE		   = 16
WARM_UP_BATCHES  = 0
TRUE_BOX_BUFFER  = 50



def sigmoid(x):
		return 1. / (1. + np.exp(-x))
		
def softmax(x):
		return(np.exp(x)/(np.exp(x)).sum(-1, keepdims=True))

def iou(b1,b2):
		x1_min=b1.x-b1.w/2
		x1_max=b1.x+b1.w/2
		y1_min=b1.y-b1.h/2
		y1_max=b1.y+b1.h/2
		x2_min=b2.x-b2.w/2
		x2_max=b2.x+b2.w/2
		y2_min=b2.y-b2.h/2
		y2_max=b2.y+b2.h/2
		x_intersect=0
		y_intersect=0
		if((x1_min>x2_min)&(x1_min<x2_max)):
				x_intersect=x2_max-x1_min-max(0,x2_max-x1_max)
		elif((x2_min>x1_min)&(x2_min<x1_max)):
				x_intersect=x1_max-x2_min-max(0,x1_max-x2_max)
		if((y1_min>y2_min)&(y1_min<y2_max)):
				y_intersect=y2_max-y1_min-max(0,y2_max-y1_max)
		elif((y2_min>y1_min)&(y2_min<y1_max)):
				y_intersect=y1_max-y2_min-max(0,y1_max-y2_max)
		intersect=x_intersect*y_intersect
		return intersect/(b1.w*b1.h+b2.w*b2.h-intersect)


class bbox:
		def __init__(self,x,y,w,h,c,classes):
				self.x=x
				self.y=y
				self.w=w
				self.h=h
				self.c=c
				self.classes=classes
				self.label=-1
				self.score=-1
		def get_label(self):
				if self.label == -1:
						self.label = np.argmax(self.classes)

				return self.label

		def get_score(self):
				if self.score == -1:
						self.score = self.classes[self.get_label()]

				return self.score


def get_YOLO_boxes(YOLO_out,con_thre,NMS_thre,anchors):
		a=YOLO_out[0]
		Cells_x=a.shape[0]
		Cells_y=a.shape[1]
		BOXES=a.shape[2]
		CLASS=a[..., 5:].shape[3]
		a[...,4]=sigmoid(a[...,4])
		#print(a[...,4]>con_thre)
		a[...,5:]=np.expand_dims(a[...,4],-1)*softmax(a[...,5:])
		a[...,5:][a[...,5:]<con_thre]=0
		bboxes=[]
		for r in range(Cells_x):
				for c in range(Cells_y):
						for BOX in range(BOXES):
								classes=a[r,c,BOX,5:]
								confidence=a[r,c,BOX,4]
								#print(confidence)
								if ((classes.sum()!=0)&(confidence> con_thre)):
										#print('hi')
										x=(c+sigmoid(a[r,c,BOX,0]))/Cells_x
										y=(r+sigmoid(a[r,c,BOX,1]))/Cells_y
										w=(anchors[2*BOX]*np.exp(a[r,c,BOX,2]))/Cells_x
										h=(anchors[2*BOX+1]*np.exp(a[r,c,BOX,3]))/Cells_y
										bboxes.append(bbox(x,y,w,h,confidence,classes))

		return bboxes


		#NMS								
def NMS(bboxes,YOLO_out,con_thre,NMS_thre,anchors):
		a=YOLO_out[0]
		Cells_x=a.shape[0]
		Cells_y=a.shape[1]
		BOXES=a.shape[2]
		CLASS=a[..., 5:].shape[3]
		for c in WANTED_IDXS:
				sorted_class_indices = list(reversed(np.argsort([bbox.classes[c] for bbox in bboxes])))
				for i in range(len(sorted_class_indices)):
						if bboxes[sorted_class_indices[i]].classes[c]==0:
								continue
						for j in range (i+1,len(sorted_class_indices)):
								if bboxes[sorted_class_indices[j]].classes[c]==0:
										continue
								if iou(bboxes[sorted_class_indices[i]],bboxes[sorted_class_indices[j]])>NMS_thre:
										bboxes[sorted_class_indices[j]].classes[c]=0

		bboxes = [box for box in bboxes if ((box.get_score() > con_thre)&(box.get_label() in WANTED_IDXS))]
		
		return bboxes
		
def draw_bboxes(image,bboxes,labels,wanted_labels):
		if bboxes==[]:
				return None
		
		bboxes=sorted(bboxes, key=lambda bbox: bbox.get_score())		
		box=bboxes[-1]
		xmin=int((box.x-box.w/2)*image.shape[1])
		xmax=int((box.x+box.w/2)*image.shape[1])
		ymin=int((box.y-box.h/2)*image.shape[0])
		ymax=int((box.y+box.h/2)*image.shape[0])
		
		xmin=max(0,xmin)
		xmin=min(image.shape[1]-1,xmin)
		
		xmax=max(0,xmax)
		xmax=min(image.shape[1]-1,xmax)
		
		ymin=max(0,ymin)
		ymin=min(image.shape[0]-1,ymin)
		
		ymax=max(0,ymax)
		ymax=min(image.shape[0]-1,ymax)
		
		to_classify_image=image[ymin:ymax,xmin:xmax,:]
		return to_classify_image


def tiny_yolo_detect(image,model):
		img=image
		image=cv2.resize(image, (416, 416))
		image=(np.array(image,dtype=np.float)/255)
		image = np.expand_dims(image, 0)
		a=model.predict(image)
		bboxes=get_YOLO_boxes(a,OBJ_THRESHOLD,NMS_THRESHOLD,ANCHORS)
		bboxes=NMS(bboxes,a,OBJ_THRESHOLD,NMS_THRESHOLD,ANCHORS)
		draw_image=draw_bboxes(img,bboxes,LABELS,WANTED_LABELS)
		return draw_image
