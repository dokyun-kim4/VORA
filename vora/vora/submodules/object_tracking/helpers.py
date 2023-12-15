import numpy as np
import pandas as pd
from .sort import Sort

objects = {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 8: 'boat',
            9: 'traffic light', 10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat',
            16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
            25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 32: 'sports ball', 
            33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
            40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 
            49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 
            58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 
            67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 
            76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
        }

object_sortkey = ['bottle','cup','mouse','cell phone','book','scissors']

sort1 = Sort()
sort2 = Sort()
sort3 = Sort()
sort4 = Sort()
sort5 = Sort()
sort6 = Sort()

sorter = [sort1,sort2,sort3,sort4,sort5,sort6]


def convert_to_df(results)->pd.DataFrame:
    """
    A helper function for extracting confidence and bounding box information
    from detected objects.

    Args:
        results (list): output from running YOLO model on a video frame
    
    Returns:
        df (pd.Dataframe): object name, bbox coords, and confidence level organized into a dataframe format
    """

    bounding_box = results[0].boxes
    box_xyxy = bounding_box.xyxy.cpu().numpy()
    box_conf = bounding_box.conf.cpu().numpy()
    obj_ids = bounding_box.cls.cpu().numpy()
    box_name = []
    for id in obj_ids:
        box_name.append(objects[id])
    # box_name = sorted(box_name, key = lambda x: object_sortkey.index(x))
    
    sort_order = object_sortkey

    df = pd.DataFrame({
        'name': box_name,
        'bbox':box_xyxy.tolist(),
        'conf':box_conf.tolist()
        })
    
    df['name'] = pd.Categorical(df['name'], categories=sort_order, ordered=True)
    df = df.sort_values(by='name')
    df = df.reset_index(drop=True)
    df_filtered  = df[df['conf'] > 0.5]
    return df_filtered

def get_obj_from_df(data: pd.DataFrame, name: str):
    bboxes = []
    confs = []
    objs = data[data['name'] == name]

    for _,rows in objs.iterrows():
        bboxes.append(rows.bbox)
        confs.append(rows.conf)

    conf_box = np.array([[int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3]),confs[i]] for i,bbox in enumerate(bboxes)])

    return conf_box


def get_tracks(results):

    result_df = convert_to_df(results)
    conf_box_all = {}
    for name in object_sortkey:
        conf_box = get_obj_from_df(result_df,name)
        conf_box_all[name] = conf_box

    track_all = {}
    for i,name in enumerate(object_sortkey):
        crnt_conf_box = conf_box_all[name]
        if len(crnt_conf_box) != 0:
            track_all[name] = sorter[i].update(crnt_conf_box)
        else:

            track_all[name] = sorter[i].update()
    
    return track_all


def find_obj(name: str, tracks: dict):
    """
    Given the object's name, return the location of the closest object of that type from the dataframe.

    Args:
        name (str): name of object
        tracks: dictionary containing bounding box coordinates of different objects and id
    
    Returns:
        (x,y) (tuple): x and y coordinates of the center of object's bounding box
    """

    try:
        bbox = tracks[name][0]
        x = int((bbox[0] + bbox[2])/2)
        y = int((bbox[1] + bbox[3])/2)
        return (x,y)
    except IndexError:
        return None


def stop(name,tracks):
    bbox = tracks[name][0]
    print(bbox[3])
    y_bottom_corner = bbox[3]
    if y_bottom_corner > 475:
        return True
    else:
        return False
