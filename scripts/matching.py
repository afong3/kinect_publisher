import cv2 as cv 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Testing image_matching
from skimage.io import imread, imshow
from skimage.feature import match_template
from skimage.io import imread, imshow


def orb(path_query, path_train):
    img1 = cv.imread(path_query,cv.IMREAD_GRAYSCALE) # queryImage
    img2 = cv.imread(path_train,cv.IMREAD_GRAYSCALE) # trainImage
    
    # Initiate ORB detector
    orb = cv.ORB_create()
    
    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    # create BFMatcher object
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    
    # Match descriptors.
    matches = bf.match(des1,des2)
    
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
    
    # Draw first 10 matches.
    img3 = cv.drawMatches(img1,kp1,img2,kp2,matches[:50],None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    plt.imshow(img3),plt.show()

def sift(path_query, path_train):
    # Initiate SIFT detector
    sift = cv.SIFT_create()

    img1 = cv.imread(path_query, cv.IMREAD_GRAYSCALE)
    img2 = cv.imread(path_train, cv.IMREAD_GRAYSCALE)
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # BFMatcher with default params
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)

    # Apply ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            good.append([m])

    # cv.drawMatchesKnn expects list of lists as matches.
    img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    plt.imshow(img3),plt.show()

def inliers_indices(data, m = 1.5):
    q75, q25 = np.percentile(data, [75, 25])
    median = np.median(data)
    iqr = q75 - q25
    return np.logical_and(data > (median - m * iqr), data < (median + m * iqr))

def flann(path_no_context, path_context):
    img_no_context = cv.imread(path_no_context,cv.IMREAD_GRAYSCALE) # queryImage
    img_in_context = cv.imread(path_context,cv.IMREAD_GRAYSCALE) # trainImage
    
    # Initiate SIFT detector
    sift = cv.SIFT_create()
    
    # find the keypoints and descriptors with SIFT
    kp1, des = sift.detectAndCompute(img_no_context,None) # query
    kp_in_context, des_in_context = sift.detectAndCompute(img_in_context,None) # train
    
    # FLANN parameters
    FLANN_INDEX_KDTREE = 3
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=200) # or pass empty dictionary
    
    flann = cv.FlannBasedMatcher(index_params,search_params)
    
    matches = flann.knnMatch(des,des_in_context,k=2)
    
    # Get the points of the good matches in the query image
    good_matches_query = []
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.7*n.distance: # higher the value the more they let through
            good_matches_query.append(m)
    
    img2_col = cv.cvtColor(img_in_context, cv.COLOR_GRAY2RGB)
    pts = []
    for match in good_matches_query:
        pt = kp_in_context[match.trainIdx].pt
        pts.append(pt)
        cv.circle(img2_col, np.int32(pt), 5, (255, 0, 0), -1)
   
    pts_x = np.array(pts)[:,0]
    x_inliers = inliers_indices(pts_x)
    # pts_y = np.array(pts)[:,1]
    # y_inliers = inliers_indices(pts_y)

    x_min = np.min(pts_x[x_inliers])
    x_max = np.max(pts_x[x_inliers])

    # Make a rectangle the entire height of the image
    tl = (x_min,0)
    tr = (x_max,img_in_context.shape[0])

    cv.rectangle(img2_col, np.int32(tl), np.int32(tr), (255, 255, 0), -1, )
    
    cv.namedWindow("test", cv.WINDOW_NORMAL) 
    cv.imshow("test", img2_col)
    cv.waitKey()

class Matcher:
    def __init__(self, paths: list, object_types):
        FLANN_INDEX_KDTREE = 3
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=200) # or pass empty dictionary
        self.sift = cv.SIFT_create()
        self.flann = cv.FlannBasedMatcher(index_params,search_params)
        self.object_descriptors = []
        self.object_types = object_types

        for path in paths:
            img = cv.imread(path, cv.IMREAD_GRAYSCALE)
            kp, des = self.sift.detectAndCompute(img,None)
            self.object_descriptors.append(des)

    def match(self, img_in_context):
            kp, des2 = self.sift.detectAndCompute(img_in_context,None) # query
            img_col = cv.cvtColor(img_in_context.copy(), cv.COLOR_GRAY2RGB)

            for j, obj_des in enumerate(self.object_descriptors):
                matches = self.flann.knnMatch(obj_des, des2, k=2)

                # Get the points of the good matches in the query image
                good_matches_query = []
                # ratio test as per Lowe's paper
                for _, (m,n) in enumerate(matches):
                    if m.distance < 0.7*n.distance: # higher the value the more they let through
                        good_matches_query.append(m)
                
                if len(good_matches_query) < 50:
                    continue

                pts = []

                col = (0, 0, 0)
                if self.object_types[j] == "box":
                    col = (255, 0, 0)
                elif self.object_types[j] == "cup":
                    col = (0, 255, 0)   
                elif self.object_types[j] == "fries":
                    col = (0, 0, 255)   

                for match in good_matches_query:
                    pt = kp[match.trainIdx].pt
                    
                    pts.append(pt)
                    cv.circle(img_col, np.int32(pt), 5, col, -1)
            
                pts_x = np.array(pts)[:,0]
                x_inliers = inliers_indices(pts_x, 1.5)
                # pts_y = np.array(pts)[:,1]
                # y_inliers = inliers_indices(pts_y)

                x_min = np.min(pts_x[x_inliers])
                x_max = np.max(pts_x[x_inliers])

                # Make a rectangle the entire height of the image
                tl = (x_min,0)
                br = (x_max,img_in_context.shape[0])

                cv.rectangle(img_col, np.int32(tl), np.int32(br), col, 4, )
            
            cv.namedWindow("test", cv.WINDOW_NORMAL) 
            cv.imshow("test", img_col)
            cv.waitKey()

if __name__ == "__main__":
    # orb("box_3.jpg", "query_3.jpg")
    # sift("box_3.jpg", "query_3.jpg")
    
    ## Works decently well
    # flann("box_3.jpg", "query_2.jpg")
    # flann("cup_3.jpg", "query_2.jpg")
    # flann("fries_2.jpg", "query_2.jpg")

    paths = ["box_3.jpg", "cup_3.jpg", "fries_2.jpg"]
    matcher = Matcher(paths, ["box", "cup", "fries"])
    matcher.match(cv.imread("query_1.jpg", cv.IMREAD_GRAYSCALE))
