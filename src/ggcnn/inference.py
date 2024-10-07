import argparse
import logging
import matplotlib.pyplot as plt
import os
from utils.dataset_processing import grasp
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import torch.utils.data

from models.common import post_process_output
from utils.dataset_processing import evaluation, grasp
from utils.data import get_dataset

logging.basicConfig(level=logging.INFO)
dataset = 'cornell'
dataset_path = r'/home/venk/VBM_project/ggcnn/cornell_grasp_dataset_01/'
network_path = r'/home/venk/VBM_project/ggcnn/ggcnn_weights_cornell/ggcnn_epoch_23_cornell'

if __name__ == '__main__':
      net = torch.load(network_path)
      device = torch.device("cuda:0")
      logging.info('Loading {} Dataset...'.format('Cornell'))
      Dataset = get_dataset(dataset)

      test_dataset = Dataset(dataset_path)

      test_data = torch.utils.data.DataLoader(
        test_dataset,
        batch_size=1,
        shuffle=False,
      )
      logging.info('Done')
      results = {'correct': 0, 'failed': 0}

      with torch.no_grad():
        for idx, (x, y, didx, rot, zoom) in enumerate(test_data):
            logging.info('Processing {}/{}'.format(idx+1, len(test_data)))
            xc = x.to(device)
            yc = [yi.to(device) for yi in y]
            lossd = net.compute_loss(xc, yc)

            q_img, ang_img, width_img = post_process_output(lossd['pred']['pos'], lossd['pred']['cos'],
                                                        lossd['pred']['sin'], lossd['pred']['width'])
            
            print("q_img, ang_img, width_img",type(q_img), q_img.shape, ang_img.shape, width_img.shape)
            gs = grasp.detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=1)
            fig = plt.figure(figsize=(10, 10))
            plt.ion()
            plt.clf()
            ax = plt.subplot(111)
            ax.imshow(q_img)
            for g in gs:
                  g.plot(ax)
            ax.set_title('Quality')
            ax.axis('off')
            fig.savefig(f'/home/venk/VBM_project/q_image_{idx}.png')

            fig = plt.figure(figsize=(10, 10))
            plt.ion()
            plt.clf()
            ax = plt.subplot(111)
            ax.imshow(ang_img)
            for g in gs:
                  g.plot(ax)
            ax.set_title('Quality')
            ax.axis('off')
            fig.savefig(f'/home/venk/VBM_project/ang_image_{idx}.png')

            fig = plt.figure(figsize=(10, 10))
            plt.ion()
            plt.clf()
            ax = plt.subplot(111)
            ax.imshow(width_img)
            for g in gs:
                  g.plot(ax)
            ax.set_title('Quality')
            ax.axis('off')
            fig.savefig(f'/home/venk/VBM_project/width_image_{idx}.png')
            # plt.imshow(q_img)
            # plt.savefig(f'/home/venk/VBM_project/q_image_{idx}.png')
            # plt.imshow(ang_img)
            # plt.savefig(f'/home/venk/VBM_project/ang_image_{idx}.png')
            # plt.imshow(width_img)
            # plt.savefig(f'/home/venk/VBM_project/width_image_{idx}.png')

            s = evaluation.calculate_iou_match(q_img, ang_img, test_data.dataset.get_gtbb(didx, rot, zoom),
                                                   no_grasps=1,
                                                   grasp_width=width_img,
                                                   )
            if s:
                  results['correct'] += 1
            else:
                  results['failed'] += 1

            logging.info('IOU Results: %d/%d = %f' % (results['correct'],
                              results['correct'] + results['failed'],
                              results['correct'] / (results['correct'] + results['failed'])))
