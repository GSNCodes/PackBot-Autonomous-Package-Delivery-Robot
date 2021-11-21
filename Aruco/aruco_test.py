import argparse
import imutils
from imutils.video import VideoStream
import time
import cv2
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# all tags supported by cv2
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_6X6_250"])
arucoParams = cv2.aruco.DetectorParameters_create()
board = cv2.aruco.CharucoBoard_create(6, 4, 1, 0.8, arucoDict)


# imboard = board.draw((4000, 4000))
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# plt.imshow(imboard, cmap = "gray", interpolation = "nearest")
# ax.axis("off")
# #cv2.imwrite(imagesFolder + "/chessboard.tiff",imboard)
# plt.savefig("./chessboard.pdf")
# plt.grid()
# plt.show()

# fig = plt.figure()
# nx = 6
# ny = 4
# for i in range(1, nx*ny+1):
#     ax = fig.add_subplot(ny,nx, i)
#     img = cv2.aruco.drawMarker(arucoDict,i, 700)
#     plt.imshow(img, cmap = "gray", interpolation = "nearest")
#     ax.axis("off")

# # plt.savefig("newboard.pdf")
# plt.show()

def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im, 0)
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        res = cv2.aruco.detectMarkers(frame, arucoDict)

        if len(res[0])>0:
            res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],frame,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = frame.shape
    return allCorners,allIds,imsize

def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

def detectFromStream(size_of_marker, mtx, dist):
    vs = VideoStream(src=0).start()
    time.sleep(2.0)

    # loop over the frames from the video stream
    while True:
        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 1000 pixels
        frame = vs.read()
        #frame = imutils.resize(frame, width=1000)
        # plt.imshow(frame)
        # plt.show()
        # continue
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)
            # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            i = 0
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                cv2.putText(frame, str(int(np.linalg.norm(tvecs[i]))) + "cm",
                    (topLeft[0], topLeft[1] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)
                
                i += 1

        # if tvecs is not None:
        #     for i,v in enumerate(tvecs):
        #         print(ids[i], ": ", np.linalg.norm(v))

        # plt.imshow(frame)
        # plt.show()
        # continue
        
        # show the output frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(.1)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
    # do a bit of cleanup
    cv2.destroyAllWindows()
    vs.stop()

if __name__ == "__main__":
    #detectFromStream()
    imagesFolder = "./images/"
    images = [imagesFolder + f for f in os.listdir(imagesFolder) if f.startswith("img_")]

    allCorners, allIds, imsize = read_chessboards(images)

    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)

    size_of_marker = 3.9
    detectFromStream(size_of_marker, mtx, dist)
