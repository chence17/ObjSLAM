'''
Author: Antonio Chan
Date: 2021-04-11 10:33:46
LastEditTime: 2021-05-05 17:30:27
LastEditors: Antonio Chan
Description: Description
FilePath: /undefined/home/antonio/Desktop/ObjSLAM/Detection/src/server.py
'''

# https://blog.csdn.net/shyjhyp11/article/details/109891396

from utils.visualization_utils import show_image_with_boxes, merge_rgb_to_bev, predictions_to_kitti_format
from utils.misc import time_synchronized
from utils.evaluation_utils import post_processing, rescale_boxes, post_processing_v2
from utils.misc import make_folder
from models.model_utils import create_model
from data_process.kitti_dataloader import create_test_dataloader
from data_process.kitti_dataloader import create_test_dataset
from data_process import kitti_data_utils, kitti_bev_utils
import config.kitti_config as cnf
import socket
import argparse
import sys
import os
import time

from easydict import EasyDict as edict
import cv2
import torch
import numpy as np

import json  # NOTE: 输出json的代码

sys.path.append('../')


# NOTE: 输出json的代码

class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """

    def default(self, obj):
        if isinstance(obj, (np.int_, np.intc, np.intp, np.int8,
                            np.int16, np.int32, np.int64, np.uint8,
                            np.uint16, np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.float_, np.float16, np.float32,
                              np.float64)):
            return float(obj)
        elif isinstance(obj, (np.ndarray,)):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def parse_test_configs():
    parser = argparse.ArgumentParser(
        description='Demonstration config for Complex YOLO Implementation')
    parser.add_argument('--saved_fn', type=str, default='complexer_yolov4', metavar='FN',
                        help='The name using for saving logs, models,...')
    parser.add_argument('-a', '--arch', type=str, default='darknet', metavar='ARCH',
                        help='The name of the model architecture')
    parser.add_argument('--cfgfile', type=str, default='./config/cfg/complex_yolov4.cfg', metavar='PATH',
                        help='The path for cfgfile (only for darknet)')
    parser.add_argument('--pretrained_path', type=str, default=None, metavar='PATH',
                        help='the path of the pretrained checkpoint')
    parser.add_argument('--use_giou_loss', action='store_true',
                        help='If true, use GIoU loss during training. If false, use MSE loss for training')

    parser.add_argument('--no_cuda', action='store_true',
                        help='If true, cuda is not used.')
    parser.add_argument('--gpu_idx', default=None, type=int,
                        help='GPU index to use.')

    parser.add_argument('--img_size', type=int, default=608,
                        help='the size of input image')
    parser.add_argument('--num_samples', type=int, default=None,
                        help='Take a subset of the dataset to run and debug')
    parser.add_argument('--num_workers', type=int, default=1,
                        help='Number of threads for loading data')
    parser.add_argument('--batch_size', type=int, default=1,
                        help='mini-batch size (default: 4)')

    parser.add_argument('--conf_thresh', type=float, default=0.5,
                        help='the threshold for conf')
    parser.add_argument('--nms_thresh', type=float, default=0.5,
                        help='the threshold for conf')

    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help='host address')
    parser.add_argument('--port', type=int, default=6783,
                        help='server port')
    parser.add_argument('--max_length', type=int, default=16384,
                        help='maximum length of string')

    configs = edict(vars(parser.parse_args()))
    configs.pin_memory = True

    ####################################################################
    ##############Dataset, Checkpoints, and results dir configs#########
    ####################################################################
    configs.working_dir = '../'
    configs.dataset_dir = os.path.join(configs.working_dir, 'dataset', 'kitti')

    return configs


class SocketServer:
    def __init__(self):
        self.configs = parse_test_configs()
        self.configs.distributed = False  # For testing

        host = self.configs.host
        port = self.configs.port

        print("> server start.... ")
        socketer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # set the port reuesd
        socketer.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        socketer.bind((host, port))
        # define the max connection
        socketer.listen(10)
        self.sock = socketer
        # Model Initial
        self.model = create_model(self.configs)
        self.model.print_network()
        print('\n\n' + '-*=' * 30 + '\n\n')

        device_string = 'cpu' if self.configs.no_cuda else 'cuda:{}'.format(
            self.configs.gpu_idx)

        assert os.path.isfile(self.configs.pretrained_path), "No file at {}".format(
            self.configs.pretrained_path)
        # model.load_state_dict(torch.load(configs.pretrained_path))
        self.model.load_state_dict(torch.load(
            self.configs.pretrained_path, map_location=device_string))

        # configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))
        self.configs.device = torch.device(device_string)
        self.model = self.model.to(device=self.configs.device)

        self.model.eval()

        # self.test_dataloader = create_test_dataloader(self.configs)
        # self.test_dataloader_iter = self.test_dataloader._get_iterator()
        self.test_dataset = create_test_dataset(self.configs)
        self.batch_idx = 0
        self.need_create_window = True

    def start_server(self):
        while True:
            print("> waiting for connection....")
            client, address = self.sock.accept()
            print("> new connection :  IP: {0}; Port:{1} ".format(
                address[0], address[1]))
            #t = threading.Thread(target=self.client_recv,args=(client,address))
            # t.start()
            self.client_recv(client, address)
            print(">  -------Done for this client. -------")
            cv2.destroyAllWindows()
            self.need_create_window = True

    def client_recv(self, client, address):
        while True:
            # read message from socket
            # client_msg_0\x00\x00\x00\x00\x00...
            msg = client.recv(1024).decode("utf-8")
            msg = msg.rstrip("\x00")
            if msg == '':
                return
            if msg == "EOF":
                return
            elif msg == "quit_client":
                client.close()
                # self.sock.close()
                print("> client  exit...")
                return
            elif msg == "quit_server":
                client.close()
                self.sock.close()
                print("> server  exit...")
                sys.exit(0)
            else:
                print("> -------", time.strftime('%Y-%m-%d %H:%M:%S',
                      time.localtime(time.time())), "-------")
                print("> receive the msg from client : {0}".format(msg))
                print('> inference for {0}'.format(msg))
                if(self.need_create_window):
                    # NOTE ObjSLAM
                    cv2.namedWindow("YOLO", flags=cv2.WINDOW_GUI_NORMAL)
                    self.need_create_window = False
                # Inference
                with torch.no_grad():
                    # img_paths, imgs_bev = self.test_dataloader_iter.next()
                    img_paths, imgs_bev = self.test_dataset[int(msg)]
                    img_paths = [img_paths]
                    imgs_bev = torch.from_numpy(
                        np.expand_dims(imgs_bev, axis=0))
                    input_imgs = imgs_bev.to(
                        device=self.configs.device).float()
                    outputs = self.model(input_imgs)
                    detections = post_processing_v2(
                        outputs, conf_thresh=self.configs.conf_thresh, nms_thresh=self.configs.nms_thresh)

                    img_detections = []  # Stores detections for each image index
                    img_detections.extend(detections)

                    img_bev = imgs_bev.squeeze() * 255
                    img_bev = img_bev.permute(1, 2, 0).numpy().astype(np.uint8)
                    img_bev = cv2.resize(
                        img_bev, (self.configs.img_size, self.configs.img_size))
                    for detections in img_detections:
                        if detections is None:
                            continue
                        # Rescale boxes to original image
                        detections = rescale_boxes(
                            detections, self.configs.img_size, img_bev.shape[:2])
                        for x, y, w, l, im, re, *_, cls_pred in detections:
                            yaw = np.arctan2(im, re)
                            # Draw rotated box
                            kitti_bev_utils.drawRotatedBox(
                                img_bev, x, y, w, l, yaw, cnf.colors[int(cls_pred)])

                    img_rgb = cv2.imread(img_paths[0])
                    calib = kitti_data_utils.Calibration(img_paths[0].replace(
                        ".png", ".txt").replace("image_2", "calib"))
                    objects_pred = predictions_to_kitti_format(
                        img_detections, calib, img_rgb.shape, self.configs.img_size)
                    # NOTE: 输出json的代码
                    frame_object_list = []
                    for i in objects_pred:
                        frame_object = dict()
                        frame_object['type'] = i.type
                        frame_object['center'] = i.t
                        frame_object['length'] = i.l
                        frame_object['width'] = i.w
                        frame_object['height'] = i.h
                        frame_object['theta'] = i.ry
                        box3d_pts_2d, _ = kitti_data_utils.compute_box_3d(
                            i, calib.P)
                        if box3d_pts_2d is None:
                            frame_object['box3d_pts_2d'] = box3d_pts_2d
                        elif box3d_pts_2d.size == 16:
                            frame_object['box3d_pts_2d'] = box3d_pts_2d
                        else:
                            frame_object['box3d_pts_2d'] = box3d_pts_2d[:8, :]
                        frame_object_list.append(frame_object)
                    result = json.dumps(frame_object_list, cls=NumpyEncoder)
                    img_bev = cv2.flip(cv2.flip(img_bev, 0), 1)
                    scale = 1.5
                    cv2.resizeWindow("YOLO",
                                     width=int(img_bev.shape[1] * scale),
                                     height=int(img_bev.shape[0] * scale))
                    cv2.imshow('YOLO', img_bev)
                    cv2.waitKey(10)
                    self.batch_idx += 1
                if len(result) > self.configs.max_length:
                    print("> WARNING: STRING IS TOO LONG! (MAX_LENGTH {0})".format(
                        self.configs.max_length))
                client.send(result.encode(encoding='utf-8'))
                print("> send the responce back to client, string length: {0}".format(
                    len(result)))
        return


if __name__ == '__main__':
    t = SocketServer()
    t.start_server()
