#!/usr/bin/env python
'''
    Logistic Regression
    To solve for a pixel-wise logistic regression model, do: 

    python logist_reg.py trainimname trainmaskname testimname --testmask testmaskname

    Once you have solved for these parameters, you can apply them to an image with
    the apply() function.  This outputs a probability image for pixels in the image.

    Daniel Morris, April 2020
    Copyright 2020
'''
import os
import numpy as np
import argparse
import cv2
from scipy.special import expit  # Sigmoid function

def plotClassifcation( img, mask, pixProbs, threshold=0.5, savename='', outfolder=''):
    ''' Plot Classification Image Results, and output to files '''
    cv2.imshow("Train Image", img)
    cv2.imshow("Test Output", pixProbs )
    if mask.any():
        cv2.imshow("Train Mask", mask)

        #Create colored scoring image:
        TP = np.logical_and( mask > 0, pixProbs > threshold )   # green
        FP = np.logical_and( mask == 0, pixProbs > threshold )  # red
        FN = np.logical_and( mask > 0, pixProbs <= threshold )  # blue
        #add gray color if any of the above labels to reduce contrast slightly
        alabel = TP + FP + FN    
        # R,G,B classification for FP, TP, FN:
        eimg = np.stack( (FN, TP, FP), axis=2 ).astype(np.uint8) * 180 + 75 * alabel[:,:,None].astype(np.uint8)
        # Superimpose this on grayscale image:
        gimg = img.mean(axis=2).astype(np.uint8)
        combimg = (eimg * 3.0/5.0 + gimg[:,:,None] * 2.0/5.0).astype(np.uint8)
        cv2.imshow("Test Scoring", combimg)
    if outfolder:
        os.makedirs(args.outpath,exist_ok=True)
        cv2.imwrite(os.path.join(outfolder,'prob_'+savename), np.uint8(pixProbs*255) )
        if mask.any():
            cv2.imwrite(os.path.join(outfolder,'scoring_'+savename), combimg )

def plotTargets(img, target_mask, centroids, savename='', outfolder=''):
    ''' Plot detected target_mask and output to file
        img: image (NxMx3) numpy array
        target_mask: (NxM) numpy array, or else empty list
        centroids: list of [x,y] centroids
    '''
    if isinstance(target_mask,list):
        target_mask = np.array(target_mask)  # Needs to be a numpy array
    if target_mask.size:  # Check if not empty numpy array:
        # Then highlight the detected pixels in the original image
        green = np.zeros_like(img)
        green[:,:,1] = 128
        mask = target_mask[:,:,None].repeat(3,axis=2)
        outim = img.copy() * (1-mask) + (img.copy()//2 + green) * mask
    else:
        outim = img.copy()  
    for centroid in centroids:
        loc = tuple( np.array(centroid).astype(int) )  # location needs to be int tuple
        cv2.circle(outim, loc, 5, (0,0,255), -1 )
    cv2.imshow("Target", outim)
    if outfolder:
        os.makedirs(args.outpath,exist_ok=True)
        cv2.imwrite(os.path.join(outfolder,'target_'+savename), outim )

def imread_channel( filename ):
    ''' Read in image and return first channel '''
    img0 = cv2.imread( filename )
    if img0 is None:
        print('Warning, unable to read:', filename)
    if len(img0.shape)==3:
        img0 = img0[:,:,0]
    return img0

class LogisticReg:
    def __init__(self ):
        ''' Initialize class with zero values '''
        self.cvec = np.zeros( (1,3) )        
        self.intercept = np.zeros( (1,) )

    def set_model(self, cvec, intercept):
        ''' Set model parameters manually
            cvec:      np.array([[p1,p2,p3]])
            intercept: np.array([p4])
        '''
        self.cvec = cvec
        self.intercept = intercept

    def fit_model_to_files(self, img_name, mask_name, exmask_name=''):
        ''' Load images from files and fit model parameters '''
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
        ''' Do logistic regression to discriminate points in non-zero region of 
            mask from other points in mask and save estimated logistic regression parameters
            exmask: optionally exclude some pixels from image '''
        # Only need sklearn when we fit model parameters -- Do not require this for inference, see apply()
        from sklearn.linear_model import LogisticRegression
        data = img.reshape((-1,3)).astype(float)   # Reshape to N x 3
        label = (mask.ravel()>0).astype(int)       # Reshape to length N 
        if exmask.any():                    # Optionally exclude pixels
            keep = exmask.ravel()==0
            data = data[keep,:]
            label = label[keep]
        sk_logr = LogisticRegression(class_weight='balanced',solver='lbfgs')
        sk_logr.fit( data, label)
        self.cvec      = sk_logr.coef_                # Extract coefficients
        self.intercept = np.array(sk_logr.intercept_) # Extract intercept

    def print_params(self):
        print('Logist Regression params, cvec:',self.cvec,'intercept:',self.intercept)

    def apply(self, img):
        ''' Application of trained logisitic regression to an image
            img:         [MxNx3] input 3-channel color image
            prob_target: [MxN] output float probability that each pixel is on target
        '''
        # Lab 4: complete this function 
        # Hint: Don't iterate over pixels, rather instead use numpy's broadcasting.  
        #
        prob_target = np.inner(self.cvec, img.reshape(-1, 3)) + self.intercept

        prob_target = expit(prob_target.reshape(img.shape[0], img.shape[1]))
        return prob_target

    def find_largest_target(self, prob_target, threshold=0.5, minpix=20):
        ''' Finds largest contiguous target region
            This takes the output of logistic regression, thresholds it
            and finds the largest contiguous region and returns its centroid
            Useful for simple cases where you are sure that the target is the
            largest object in the image of its color.
            prob_target: [MxN] input probability of target
            centroid:    [x,y] coordinates of the largest centroid
            area:        number of pixels in target
            target_mask: [MxN] binary mask with 1's at target
        '''
        # Lab 4: complete this function 
        # Hint 1: Use cv2.connectedComponentsWithStats()
        # Hint 2: Use number of pixels in region to find largest region where at least 99% pixels are target
        for pos, i in enumerate(prob_target):
            for pos2, j in enumerate(i):
                if j >= threshold:
                    prob_target[pos, pos2] = 1
                else:
                    prob_target[pos, pos2] = 0

        num_components, _, stats, centroids = cv2.connectedComponentsWithStats(prob_target.astype(np.uint8))

        maxval = 0
        maxsize = 0

        for i in range(1, num_components):
            if stats[i, -1] > maxsize:
                maxval = i
                maxsize = stats[i, -1]

        return centroids[maxval], maxsize, prob_target

    def find_all_targets(self, prob_target, threshold=0.5, minpix=20):
        ''' Finds contiguous target regions
            This takes the output of logistic regression, thresholds it
            and finds contiguous regions in the image of target pixels.
            prob_target: [MxN] input probability of target
            centroids:   list of [x,y] coordinates of target centroids
            areas:       list of number of pixels in each target
            target_mask: [MxN] binary mask with 1's at each target pixel
        '''
        # Lab 4: complete this function 
        # Hint 1: Use cv2.connectedComponentsWithStats()
        # Hint 2: Keep all regions with at least minpix pixels and where 99% of pixels are target
        for pos, i in enumerate(prob_target):
            for pos2, j in enumerate(i):
                if j >= threshold:
                    prob_target[pos, pos2] = 1
                else:
                    prob_target[pos, pos2] = 0

        num_components, output, stats, centroids = cv2.connectedComponentsWithStats(prob_target.astype(np.uint8))

        return_centroids = []
        return_areas = []


        for i in range(1, num_components):
            if stats[i, -1] >= minpix:
                return_centroids.append(centroids[i])
                return_areas.append(stats[i, -1])

        return return_centroids, return_areas, prob_target

if __name__ == '__main__':
    # This is a demonstration of how LogisticReg can be used
    parser = argparse.ArgumentParser(description='Logistic Regression')
    parser.add_argument('trainimg',      type=str,              help='Train image')
    parser.add_argument('trainmask',     type=str,              help='Train mask')
    parser.add_argument('testimg',       type=str,              help='Test image')
    parser.add_argument('--testmask',    type=str, default='',  help='Test mask')
    parser.add_argument('--trainexmask', type=str, default='',  help='Train pixels to exclude')
    parser.add_argument('--outpath',     type=str, default='',  help='Output folder')
    parser.add_argument('--threshold', type=float, default=0.5, help='Output folder')
    args = parser.parse_args()

    # Build color model with Logistic Regression
    logr = LogisticReg( )
    logr.fit_model_to_files( args.trainimg, args.trainmask, args.trainexmask )

    # Load test data:
    testimg = cv2.imread(args.testimg)
    if args.testmask:
        testmask = imread_channel(args.testmask)
    else:
        testmask = np.array([])

    # Apply model to test data:
    probt = logr.apply( testimg )
    logr.print_params()
    # Plot classification results:
    plotClassifcation(testimg, testmask, probt, args.threshold, os.path.basename(args.testimg), args.outpath )

    # Find largest regions as targets
    centroid, area, target_mask = logr.find_largest_target(probt, args.threshold)
    # Plot detected region and centroid:
    plotTargets(testimg, target_mask, [centroid] )
    cv2.waitKey()
    cv2.destroyAllWindows()

