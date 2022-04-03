#!/usr/bin/env python

import os
import numpy as np
import argparse
import cv2
from scipy.special import expit

def plotClassifcation( img, mask, pixProbs, threshold=0.5, savename='', outfolder=''):
    cv2.imshow("Image", img)
    cv2.imshow("Raw Output", pixProbs )
    if mask.any():
      cv2.imshow("Ground Truth Mask", mask)

      TP = np.logical_and(mask > 0, pixProbs > threshold)
      FP = np.logical_and(mask == 0, pixProbs > threshold)
      FN = np.logical_and(mask > 0, pixProbs <= threshold)
      alabel = TP + FP + FN    

      eimg = np.stack( (FN, TP, FP), axis=2 ).astype(np.uint8) * 180 + 75 * alabel[:,:,None].astype(np.uint8)
      gimg = img.mean(axis=2).astype(np.uint8)
      combimg = (eimg * 3.0/5.0 + gimg[:,:,None] * 2.0/5.0).astype(np.uint8)
      cv2.imshow("Scoring Using Mask", combimg)
        
    if outfolder:
      os.makedirs(args.outpath,exist_ok=True)
      cv2.imwrite(os.path.join(outfolder,'prob_'+savename), np.uint8(pixProbs*255) )
      if mask.any():
        cv2.imwrite(os.path.join(outfolder,'scoring_'+savename), combimg )

def plotTargets(img, target_mask, centroids, savename='', outfolder=''):
  if isinstance(target_mask,list):
    target_mask = np.array(target_mask)

  if target_mask.size:
    green = np.zeros_like(img)
    green[:,:,1] = 128
    mask = target_mask[:,:,None].repeat(3,axis=2)
    outim = img.copy() * (1-mask) + (img.copy()//2 + green) * mask
  else:
    outim = img.copy()  

  for centroid in centroids:
    loc = tuple(np.array(centroid).astype(int))
    try:
      cv2.circle(outim, loc, 5, (0,0,255), -1)
    except:
      return
        
  cv2.imshow("Target", outim)
  if outfolder:
    os.makedirs(args.outpath,exist_ok=True)
    cv2.imwrite(os.path.join(outfolder,'target_'+savename), outim )

def imread_channel( filename ):
  img0 = cv2.imread( filename )

  if img0 is None:
    print('Warning, unable to read:', filename)

  if len(img0.shape)==3:
    img0 = img0[:,:,0]

  return img0

class LogisticReg:
  def __init__(self ):
    self.cvec = np.zeros( (1,3) )        
    self.intercept = np.zeros( (1,) )

  def set_model(self, cvec, intercept):
    self.cvec = cvec
    self.intercept = intercept

  def fit_model_to_files(self, img_name, mask_name, exmask_name=''):
    img = cv2.imread( img_name )
    mask = imread_channel( mask_name )

    if img is None or mask is None:
      print('Error loading image and mask')
      print('image:', img_name)
      print('mask:', mask_name)

    if exmask_name:
      exmask = imread_channel(exmask_name)
    else:
      exmask = np.array([])
      
    self.fit_model( img, mask, exmask )

  def fit_model(self, img, mask, exmask=np.array([]) ):
    from sklearn.linear_model import LogisticRegression
    data = img.reshape((-1,3)).astype(float)
    label = (mask.ravel()>0).astype(int)

    if exmask.any():
      keep = exmask.ravel()==0
      data = data[keep,:]
      label = label[keep]
      
    sk_logr = LogisticRegression(class_weight='balanced',solver='lbfgs')
    sk_logr.fit( data, label)
    self.cvec = sk_logr.coef_
    self.intercept = np.array(sk_logr.intercept_)

  def print_params(self):
    print('Logist Regression params, cvec:',self.cvec,'intercept:',self.intercept)

  def apply(self, img):
    score = (img.astype(float) * self.cvec).sum(axis=2) + self.intercept
    prob_target = expit(score)

    return prob_target

  def find_largest_target(self, prob_target, threshold=0.5, minpix=20):
    binary_target = (prob_target>threshold).astype(np.uint8)
    cc = cv2.connectedComponentsWithStats(binary_target)
    inds = np.argsort(cc[2][:,4])
    target_mask = np.zeros_like(binary_target)
    centroid = []
    area = []

    for i in inds[::-1]:            
      if binary_target[cc[1]==i].astype(float).mean() > 0.99 and cc[2][i,4] >= minpix:
        target_mask += (cc[1]==i).astype(np.uint8)
        centroid = cc[3][i,:] 
        area = cc[2][i,4]

        break
    return centroid, area, target_mask

  def find_all_targets(self, prob_target, threshold=0.5, minpix=20):
    binary_target = (prob_target>threshold).astype(np.uint8)
    cc = cv2.connectedComponentsWithStats(binary_target)
    inds = np.argsort( cc[2][:,4] )
    target_mask = np.zeros_like(binary_target)
    centroids = []
    areas = []

    for i in inds[::-1]:            
      if binary_target[cc[1]==i].astype(float).mean() > 0.99 and cc[2][i,4] >= minpix:
        target_mask += (cc[1]==i).astype(np.uint8)
        centroids.append(cc[3][i,:])
        areas.append(cc[2][i,4])
    return centroids, areas, target_mask


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Logistic Regression')
  parser.add_argument('trainimg',      type=str,              help='Train image')
  parser.add_argument('trainmask',     type=str,              help='Train mask')
  parser.add_argument('testimg',       type=str,              help='Test image')
  parser.add_argument('--testmask',    type=str, default='',  help='Test mask')
  parser.add_argument('--trainexmask', type=str, default='',  help='Train pixels to exclude')
  parser.add_argument('--outpath',     type=str, default='',  help='Output folder')
  parser.add_argument('--threshold', type=float, default=0.5, help='Output folder')
  args = parser.parse_args()

  logr = LogisticReg( )
  logr.fit_model_to_files( args.trainimg, args.trainmask, args.trainexmask )

  testimg = cv2.imread(args.testimg)
  if args.testmask:
    testmask = imread_channel(args.testmask)
  else:
    testmask = np.array([])

  probt = logr.apply( testimg )
  logr.print_params()
  plotClassifcation(testimg, testmask, probt, args.threshold, os.path.basename(args.testimg), args.outpath )

  centroid, area, target = logr.find_largest_target(probt, args.threshold)
  plotTargets(testimg, target, [centroid] )
  cv2.waitKey()
  cv2.destroyAllWindows()
