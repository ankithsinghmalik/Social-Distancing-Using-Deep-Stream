#!/usr/bin/env python3

################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
################################################################################

import sys
sys.path.append('../')
import gi
import configparser
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
from gi.repository import GLib
from ctypes import *
import time
import sys
import numpy as np
import cv2
import math
import platform
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
from common.FPS import GETFPS
import pyds
from threading import Thread
from common.utils import long_to_int
import datetime
import os
import json
import time
import collections
from threading import Thread
import schedule
import requests
from scipy.spatial import distance
from queue import Queue
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

fps_streams={}

id_dict = {}
init = time.time()
path1 = "data"

d_time ={}
in_time = {}
count_li = {}

exd_time = 10

MAX_DISPLAY_LEN=64
MUXER_BATCH_TIMEOUT_USEC=4000000
GST_CAPS_FEATURES_NVMM="memory:NVMM"

id_dict = {}
init = time.time()
path1 = "data"
if not os.path.exists(path1):
    os.mkdir(path1)

number_sources = 0

#read config
with open("config.json", "r") as f:
    data = json.load(f)
    print(data)
sources = data["sources"]
MUXER_OUTPUT_WIDTH = data["processing_width"]
MUXER_OUTPUT_HEIGHT = data["processing_height"]
TILED_OUTPUT_WIDTH=data["tiler_width"]
TILED_OUTPUT_HEIGHT=data["tiler_height"]

def get_matrix(roi_points):
    pts1 = np.float32(roi_points[:4])
    print(pts1)
    pts2 = np.float32([[50, 50], [50, 850], [750, 850], [750, 50]])  # transformed coordinates of grid
    print(pts2)
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    return matrix

roi_points = [(113, 102), (9, 477), (634, 475), (489, 107), (300, 135), (360, 135)]
dist_not_maintained_id = {}
matrix = get_matrix(roi_points)
print(matrix)

px = lambda locs: (matrix[0][0]*locs[0]+matrix[0][1]*locs[1]+matrix[0][2])/(matrix[2][0]*locs[0]+matrix[2][1]*locs[1]+matrix[2][2])
py = lambda locs: (matrix[1][0]*locs[0]+matrix[1][1]*locs[1]+matrix[1][2])/(matrix[2][0]*locs[0]+matrix[2][1]*locs[1]+matrix[2][2])


def get_sep_dist(roi_points):
    points = [roi_points[4], roi_points[5]]
    new_points = [(int(px(locs)), int(py(locs))) for locs in points]
    sep_dist = distance.euclidean(new_points[0], new_points[1])
    return sep_dist

separation_dist = get_sep_dist(roi_points)
start_time = time.time()
alert = "social distancing not maintained"
polygon = Polygon(roi_points[:4])

def get_location(detection_list):    

    person_location = []
    obj_id_list = []
    obj_meta_list = []
    for person_dict in detection_list:
        x, y, w, h = person_dict["left"], person_dict["top"], person_dict["width"], person_dict["height"]
        loc = (int(x+w/2), int(y+h))
        # print(x,y,w,h)
        # print(loc)
        pt = Point(loc[0], loc[1])
        # print(roi_points[:4])
        # print("roi: ", polygon.contains(pt))
        if polygon.contains(pt):
            person_location.append(loc)
            obj_id_list.append(person_dict["object_id"])
            obj_meta_list.append(person_dict["obj_meta"])
    return person_location, obj_id_list, obj_meta_list

def persp_transform_person_location(person_location):
    persp_person_location = [(int(px(locs)), int(py(locs))) for locs in person_location]
    return persp_person_location

def do_fifo(queue):
    if queue.full():                            
        queue.get()
    return queue

def get_distance(persp_person_location, obj_id, obj_meta_list, camera_id):
    global dist_not_maintained_id
    send_alert = False
    count = 0
    obj_meta_final = []
    for i in range(0, len(persp_person_location)):
        for j in range(i+1, len(persp_person_location)):
            dist = distance.euclidean(persp_person_location[i], persp_person_location[j])
            # print(separation_dist, dist)
            if dist < separation_dist:
                print(separation_dist, dist)
                # print(list(dist_not_maintained_id[camera_id].queue))
                count += 1
                if obj_id[i] not in list(dist_not_maintained_id[camera_id].queue):
                    dist_not_maintained_id[camera_id] = do_fifo(dist_not_maintained_id[camera_id])
                    dist_not_maintained_id[camera_id].put(obj_id[i])
                    send_alert = True           #send alert when it is a new perpetrator
                    # if obj_meta_list[i] not in obj_meta_final:
                print("added1")
                obj_meta_final.append(obj_meta_list[i])
                if obj_id[j] not in list(dist_not_maintained_id[camera_id].queue):
                    dist_not_maintained_id[camera_id] = do_fifo(dist_not_maintained_id[camera_id])
                    dist_not_maintained_id[camera_id].put(obj_id[j])
                    send_alert = True           #send alert when it is a new perpetrator
                    # if obj_meta_list[j] not in obj_meta_final:
                print("added2")
                obj_meta_final.append(obj_meta_list[j])
            j+=1
        i+=1
    return send_alert, obj_meta_final

# tiler_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
# and update params for drawing rectangle, object information etc.
def tiler_src_pad_buffer_probe(pad,info,u_data):

    global start_time
    global dist_not_maintained_id
    global roi_points
    global id_dict
    global init
    global d_time
    global in_time 
    global count_li

    frame_number=0
    num_rects=0
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    detection_list = []
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

    
        '''
        print("Frame Number is ", frame_meta.frame_num)
        print("Source id is ", frame_meta.source_id)
        print("Batch id is ", frame_meta.batch_id)
        print("Source Frame Width ", frame_meta.source_frame_width)
        print("Source Frame Height ", frame_meta.source_frame_height)
        print("Num object meta ", frame_meta.num_obj_meta)
        '''
        frame_number=frame_meta.frame_num
        l_obj=frame_meta.obj_meta_list
        num_rects = frame_meta.num_obj_meta

        camera_id = frame_meta.pad_index
        if camera_id not in dist_not_maintained_id:
            dist_not_maintained_id[camera_id] = Queue(maxsize=20)
        # print("Num object meta ", frame_meta.num_obj_meta)
        # obj_meta_list = []


        while l_obj is not None:
            try: 
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break

            if camera_id not in id_dict:
                id_dict[camera_id] = {}
                
            if camera_id not in count_li:
                count_li[camera_id] = {}

            obj_id = obj_meta.object_id

            l_user_meta = obj_meta.obj_user_meta_list
            
            count_per = 0
            
            while l_user_meta:
                try:
                    user_meta = pyds.NvDsUserMeta.cast(l_user_meta.data)
                    
                except StopIteration:
                    break

                if user_meta.base_meta.meta_type == pyds.nvds_get_user_meta_type("NVIDIA.DSANALYTICSOBJ.USER_META"):             
                    user_meta_data = pyds.NvDsAnalyticsObjInfo.cast(user_meta.user_meta_data)
                    roistatus = user_meta_data.roiStatus
                    if roistatus:
                        roi = roistatus[0]
                        if roi not in id_dict[camera_id].keys():
                            id_dict[camera_id][roi] = []
                        
                        if roi not in count_li[camera_id].keys():
                            count_li[camera_id][roi] = []
                        
                        if obj_id not in id_dict[camera_id][roi]:
                            id_dict[camera_id][roi].append(obj_id)
                            d_time[obj_id] = time.time()
                            in_time[obj_id] = 0

                        if obj_id in id_dict[camera_id][roi]:
                            cr_time = time.time()
                            diff = cr_time - d_time[obj_id]
                            d_time[obj_id] = time.time()
                            in_time[obj_id] += diff



                        if in_time[obj_id] > exd_time and obj_id not in count_li[camera_id][roi]:
                            count_li[camera_id][roi].append(obj_id)
                    


                # print("People visited roi1: ", len(id_dict[camera_id]["CF1"]))
                

                try:
                    l_user_meta = l_user_meta.next
                except StopIteration:
                    break

            
            x = obj_meta.rect_params.left
            y = obj_meta.rect_params.top
            w = obj_meta.rect_params.width
            h = obj_meta.rect_params.height

            obj_id = obj_meta.object_id
            detection_list.append({"left": x, "top": y, "width": w, "height": h, "object_id": obj_id, "obj_meta": obj_meta})
            
            obj_meta.text_params.display_text = f'{obj_meta.object_id} Outside ROI'
            obj_meta.rect_params.border_color.red = 0
            obj_meta.rect_params.border_color.blue = 1
            obj_meta.rect_params.border_color.green = 1
            obj_meta.rect_params.border_color.alpha = 1

            try: 
                l_obj=l_obj.next
            except StopIteration:
                break           

        # Get frame rate through this probe
        fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps()

        person_location, obj_id_list, obj_meta_list = get_location(detection_list)
        persp_person_location = persp_transform_person_location(person_location)
        send_dist_alerts, obj_meta_final = get_distance(persp_person_location, obj_id_list, obj_meta_list, camera_id)
        # print(send_dist_alerts)
        if send_dist_alerts and (time.time() - start_time) > 10:
            print("Alerts sent")
            start_time = time.time()
        print(len(obj_meta_list))
        print(len(obj_meta_final))
        print("**********************************************")
        for obj_meta in obj_meta_list:
            obj_meta.text_params.display_text = f'{obj_meta.object_id} Inside ROI'
            obj_meta.rect_params.border_color.red = 0
            obj_meta.rect_params.border_color.blue = 0
            obj_meta.rect_params.border_color.green = 1
            obj_meta.rect_params.border_color.alpha = 1

        for obj_meta in obj_meta_final:
            obj_meta.text_params.display_text = f'{obj_meta.object_id} Violated'
            obj_meta.rect_params.border_color.red = 1
            obj_meta.rect_params.border_color.blue = 0
            obj_meta.rect_params.border_color.green = 0
            obj_meta.rect_params.border_color.alpha = 1

        display_meta=pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        # keys = id_dict[frame_meta.pad_index].keys()
        display_meta.num_lines = 4
        # disp = ["Social ", "Path2"]
        for i in range(len(roi_points)-2):
            # print("here")
            py_nvosd_line_params = display_meta.line_params[i]
            py_nvosd_line_params.x1 = roi_points[i][0]
            py_nvosd_line_params.y1 = roi_points[i][1]
            if i < 3:
                py_nvosd_line_params.x2 = roi_points[i+1][0]
                py_nvosd_line_params.y2 = roi_points[i+1][1]
            else:
                py_nvosd_line_params.x2 = roi_points[0][0]
                py_nvosd_line_params.y2 = roi_points[0][1]
            py_nvosd_line_params.line_color.red = 0.0
            py_nvosd_line_params.line_color.green = 1.0
            py_nvosd_line_params.line_color.blue = 0.0
            py_nvosd_line_params.line_color.alpha = 1.0
            py_nvosd_line_params.line_width = 3
        
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)
        

        py_nvosd_text_params = display_meta.text_params[0]
        py_nvosd_text_params.display_text = "RF2: {0}: ".format(,len(count_li[camera_id]["RF2"]))
        py_nvosd_text_params.x_offset = 5
        py_nvosd_text_params.y_offset = 8
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 12
        py_nvosd_text_params.font_params.font_color.red = 1.0
        py_nvosd_text_params.font_params.font_color.green = 1.0
        py_nvosd_text_params.font_params.font_color.blue = 1.0
        py_nvosd_text_params.font_params.font_color.alpha = 1.0
        py_nvosd_text_params.set_bg_clr = 1
        py_nvosd_text_params.text_bg_clr.red = 0.0
        py_nvosd_text_params.text_bg_clr.green = 0.0
        py_nvosd_text_params.text_bg_clr.blue = 0.0
        py_nvosd_text_params.text_bg_clr.alpha = 1.0

        schedule.run_pending()
        time.sleep(0.05)
        # print(id_dict[frame_meta.pad_index]
        
        try:
            l_frame=l_frame.next
        except StopIteration:
            break
        
    return Gst.PadProbeReturn.OK

def get_frame(gst_buffer, batch_id):
    n_frame=pyds.get_nvds_buf_surface(hash(gst_buffer),batch_id)
    #convert python array into numy array format.
    frame_image=np.array(n_frame,copy=True,order='C')
    #covert the array into cv2 default color format
    frame_image=cv2.cvtColor(frame_image,cv2.COLOR_RGBA2BGRA)
    return frame_image

def cb_newpad(decodebin, decoder_src_pad,data):
    print("In cb_newpad\n")
    caps=decoder_src_pad.get_current_caps()
    gststruct=caps.get_structure(0)
    gstname=gststruct.get_name()
    source_bin=data
    features=caps.get_features(0)

    # Need to check if the pad created by the decodebin is for video and not
    # audio.
    print("gstname=",gstname)
    if(gstname.find("video")!=-1):
        # Link the decodebin pad only if decodebin has picked nvidia
        # decoder plugin nvdec_*. We do this by checking if the pad caps contain
        # NVMM memory features.
        print("features=",features)
        if features.contains("memory:NVMM"):
            # Get the source bin ghost pad
            bin_ghost_pad=source_bin.get_static_pad("src")
            if not bin_ghost_pad.set_target(decoder_src_pad):
                sys.stderr.write("Failed to link decoder src pad to source bin ghost pad\n")
        else:
            sys.stderr.write(" Error: Decodebin did not pick nvidia decoder plugin.\n")

def decodebin_child_added(child_proxy,Object,name,user_data):
    print("Decodebin child added:", name, "\n")
    if(name.find("decodebin") != -1):
        Object.connect("child-added",decodebin_child_added,user_data)   
    if name.find("nvv4l2decoder") != -1:
        if is_aarch64():
            print("Seting bufapi_version\n")
            Object.set_property("bufapi-version",True)

def create_source_bin(index,uri):
    print("Creating source bin")

    # Create a source GstBin to abstract this bin's content from the rest of the
    # pipeline
    bin_name="source-bin-%02d" %index
    print(bin_name)
    nbin=Gst.Bin.new(bin_name)
    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")

    # Source element for reading from the uri.
    # We will use decodebin and let it figure out the container format of the
    # stream and the codec and plug the appropriate demux and decode plugins.
    uri_decode_bin=Gst.ElementFactory.make("uridecodebin", "uri-decode-bin")
    if not uri_decode_bin:
        sys.stderr.write(" Unable to create uri decode bin \n")
    # We set the input uri to the source element
    uri_decode_bin.set_property("uri",uri)
    # Connect to the "pad-added" signal of the decodebin which generates a
    # callback once a new pad for raw data has beed created by the decodebin
    uri_decode_bin.connect("pad-added",cb_newpad,nbin)
    uri_decode_bin.connect("child-added",decodebin_child_added,nbin)

    # We need to create a ghost pad for the source bin which will act as a proxy
    # for the video decoder src pad. The ghost pad will not have a target right
    # now. Once the decode bin creates the video decoder and generates the
    # cb_newpad callback, we will set the ghost pad target to the video decoder
    # src pad.
    Gst.Bin.add(nbin,uri_decode_bin)
    bin_pad=nbin.add_pad(Gst.GhostPad.new_no_target("src",Gst.PadDirection.SRC))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin

def main(args):

    global number_sources

    try:
        number_sources = len(data["sources"].keys())         

        if len(list(sources)) == 0:
            print("No source provided in json file")
            sys.exit()

    except Exception as e:
        print(e)
        print("Error in json file")
        sys.exit(1)

    for i in range(number_sources):
        fps_streams["stream{0}".format(i)]=GETFPS(i)

    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements */
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    pipeline = Gst.Pipeline()
    is_live = False

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")
    print("Creating streamux \n ")

    # Create nvstreammux instance to form batches from one or more sources.
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")
    pipeline.add(streammux)

    for i in range(number_sources):
        if not os.path.exists(os.path.join(path1,"stream_"+str(i))):
            os.mkdir(os.path.join(path1,"stream_"+str(i)))
        print("Creating source_bin ",i," \n ")
        uri_name=sources[str(i)]
        if uri_name.find("rtsp://") == 0 :
            is_live = True
        source_bin=create_source_bin(i, uri_name)
        if not source_bin:
            sys.stderr.write("Unable to create source bin \n")
        pipeline.add(source_bin)
        padname="sink_%u" %i
        sinkpad= streammux.get_request_pad(padname) 
        if not sinkpad:
            sys.stderr.write("Unable to create sink pad bin \n")
        srcpad=source_bin.get_static_pad("src")
        if not srcpad:
            sys.stderr.write("Unable to create src pad bin \n")
        srcpad.link(sinkpad)
    
    print("Creating Pgie \n ")
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")

    # sgie = Gst.ElementFactory.make("nvinfer", "secondary-inference")
    # if not sgie:
    #     sys.stderr.write(" Unable to create sgie \n")

    tracker = Gst.ElementFactory.make("nvtracker", "tracker")
    if not tracker:
        sys.stderr.write(" Unable to create tracker \n")

    print("Creating nvdsanalytics \n ")
    nvanalytics = Gst.ElementFactory.make("nvdsanalytics", "analytics")
    if not nvanalytics:
        sys.stderr.write(" Unable to create nvanalytics \n")
    nvanalytics.set_property("config-file", "config_nvdsanalytics.txt")

    print("Creating tiler \n ")
    tiler=Gst.ElementFactory.make("nvmultistreamtiler", "nvtiler")
    if not tiler:
        sys.stderr.write(" Unable to create tiler \n")

    print("Creating nvvidconv1 \n ")
    nvvidconv1 = Gst.ElementFactory.make("nvvideoconvert", "convertor1")
    if not nvvidconv1:
        sys.stderr.write(" Unable to create nvvidconv1 \n")

    print("Creating filter1 \n ")
    caps1 = Gst.Caps.from_string("video/x-raw(memory:NVMM), format=RGBA")
    filter1 = Gst.ElementFactory.make("capsfilter", "filter1")
    if not filter1:
        sys.stderr.write(" Unable to get the caps filter1 \n")
    filter1.set_property("caps", caps1)

    print("Creating nvvidconv \n ")
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")

    print("Creating nvosd \n ")
    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    if not nvosd:
        sys.stderr.write(" Unable to create nvosd \n")

    if(is_aarch64()):
        print("Creating transform \n ")
        transform=Gst.ElementFactory.make("nvegltransform", "nvegl-transform")
        if not transform:
            sys.stderr.write(" Unable to create transform \n")

    print("Creating EGLSink \n")
    sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
    if not sink:
        sys.stderr.write(" Unable to create egl sink \n")

    if is_live:
        print("Atleast one of the sources is live")
        streammux.set_property('live-source', 1)

    # Set properties of tracker
    config = configparser.ConfigParser()
    config.read('dstest2_tracker_config.txt')
    config.sections()

    for key in config['tracker']:
        if key == 'tracker-width':
            tracker_width = config.getint('tracker', key)
            tracker.set_property('tracker-width', tracker_width)
        if key == 'tracker-height':
            tracker_height = config.getint('tracker', key)
            tracker.set_property('tracker-height', tracker_height)
        if key == 'gpu-id':
            tracker_gpu_id = config.getint('tracker', key)
            tracker.set_property('gpu_id', tracker_gpu_id)
        if key == 'll-lib-file':
            tracker_ll_lib_file = config.get('tracker', key)
            tracker.set_property('ll-lib-file', tracker_ll_lib_file)
        if key == 'll-config-file':
            tracker_ll_config_file = config.get('tracker', key)
            tracker.set_property('ll-config-file', tracker_ll_config_file)
        if key == 'enable-batch-process':
            tracker_enable_batch_process = config.getint('tracker', key)
            tracker.set_property('enable_batch_process',
                                 tracker_enable_batch_process)

    streammux.set_property('width', MUXER_OUTPUT_WIDTH)
    streammux.set_property('height', MUXER_OUTPUT_HEIGHT)
    streammux.set_property('batch-size', number_sources)
    streammux.set_property('batched-push-timeout', 4000000)

    pgie.set_property('config-file-path', "primary_config.txt")
    pgie_batch_size=pgie.get_property("batch-size")
    if(pgie_batch_size != number_sources):
        print("WARNING: Overriding infer-config batch-size",pgie_batch_size," with number of sources ", number_sources," \n")
        pgie.set_property("batch-size",number_sources)

    # sgie.set_property('config-file-path', "./secondary_config.txt")
    # sgie.set_property("process-mode", 2)

    tiler_rows=int(math.sqrt(number_sources))
    tiler_columns=int(math.ceil((1.0*number_sources)/tiler_rows))
    tiler.set_property("rows",tiler_rows)
    tiler.set_property("columns",tiler_columns)
    tiler.set_property("width", TILED_OUTPUT_WIDTH)
    tiler.set_property("height", TILED_OUTPUT_HEIGHT)

    sink.set_property("qos",0)
    sink.set_property("sync",0)

    # nvosd.set_property('process-mode',0)
    # nvosd.set_property('display-text',0)

    if not is_aarch64():
        # Use CUDA unified memory in the pipeline so frames
        # can be easily accessed on CPU in Python.
        mem_type = int(pyds.NVBUF_MEM_CUDA_UNIFIED)
        streammux.set_property("nvbuf-memory-type", mem_type)
        nvvidconv.set_property("nvbuf-memory-type", mem_type)
        nvvidconv1.set_property("nvbuf-memory-type", mem_type)
        tiler.set_property("nvbuf-memory-type", mem_type)

    queue1=Gst.ElementFactory.make("queue","queue1")
    queue2=Gst.ElementFactory.make("queue","queue2")
    queue3=Gst.ElementFactory.make("queue","queue3")
    queue4=Gst.ElementFactory.make("queue","queue4")
    queue5=Gst.ElementFactory.make("queue","queue5")
    queue6=Gst.ElementFactory.make("queue","queue6")
    queue7=Gst.ElementFactory.make("queue","queue7")
    queue8=Gst.ElementFactory.make("queue","queue8")
    queue9=Gst.ElementFactory.make("queue","queue9")
    pipeline.add(queue1)
    pipeline.add(queue2)
    pipeline.add(queue3)
    pipeline.add(queue4)
    pipeline.add(queue5)
    pipeline.add(queue6)
    pipeline.add(queue7)
    pipeline.add(queue8)
    pipeline.add(queue9)

    print("Adding elements to Pipeline \n")
    
    pipeline.add(pgie)
    pipeline.add(tracker)
    # pipeline.add(nvanalytics)
    pipeline.add(tiler)
    pipeline.add(nvvidconv)
    pipeline.add(filter1)
    pipeline.add(nvvidconv1)
    pipeline.add(nvosd)
    if is_aarch64():
        pipeline.add(transform)
    pipeline.add(sink)

    print("Linking elements in the Pipeline \n")

    streammux.link(pgie)
    pgie.link(queue1)
    queue1.link(tracker)
    tracker.link(queue4)
    # queue2.link(nvanalytics)
    # nvanalytics.link(queue4)
    # queue3.link(sgie2)
    # sgie2.link(queue4)
    queue4.link(nvvidconv1)
    nvvidconv1.link(queue5)
    queue5.link(filter1)
    filter1.link(queue6)
    queue6.link(tiler)
    tiler.link(queue7)
    queue7.link(nvvidconv)
    nvvidconv.link(queue8)
    queue8.link(nvosd)
    if is_aarch64():
        nvosd.link(queue9)
        queue9.link(transform)
        transform.link(sink)
    else:
        nvosd.link(queue9)
        queue9.link(sink)

    # create an event loop and feed gstreamer bus messages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)

    tiler_src_pad=tiler.get_static_pad("sink")
    if not tiler_src_pad:
        sys.stderr.write(" Unable to get src pad \n")
    tiler_src_pad.add_probe(Gst.PadProbeType.BUFFER, tiler_src_pad_buffer_probe, 0)

    # List the sources
    print("Now playing...")
    for i, src in sources.items():
        print(i, ": ", src)

    print("Starting pipeline \n")
    # start play back and listed to events		
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass
    # cleanup

    print("Exiting app\n")
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    sys.exit(main(sys.argv))


