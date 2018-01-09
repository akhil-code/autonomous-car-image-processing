import numpy as np
import cv2,math


# reset global state of average values
avgLeft = (0, 0, 0, 0)
avgRight = (0, 0, 0, 0)

def seg_intersect(A,B,C,D):
    #Line AB represented as a1x + b1y = c1
    a1 = B[1] - A[1]
    b1 = A[0] - B[0]
    c1 = a1*(A[0]) + b1*(A[1])
 
    #Line CD represented as a2x + b2y = c2
    a2 = D[1] - C[1]
    b2 = C[0] - D[0]
    c2 = a2*(C[0])+ b2*(C[1])
 
    determinant = a1*b2 - a2*b1
 
    if (determinant == 0):
        #The lines are parallel. This is simplified
        return (np.inf,np.inf)
    else:
        x = (b2*c1 - b1*c2)/determinant;
        y = (a1*c2 - a2*c1)/determinant;
        return (x, y)

def movingAverage(avg, new_sample, N=20):
    if (avg == 0):
        return new_sample
    avg -= avg / N;
    avg += new_sample / N;
    return avg;

def draw_lines(img,srcImg, lines, color=[255,0,0],thickness=2):
    global avgLeft,avgRight
    print 'length: '+str(len(lines))

    # state variables to keep track of most dominant segment
    largestLeftLineSize = 0
    largestRightLineSize = 0
    largestLeftLine = (0,0,0,0)
    largestRightLine = (0,0,0,0)

    if lines is None:
        avgx1,avgy1,avgx2,avgy2=avgLeft
        cv2.line(img,(int(avgx1),int(avgy1)),(int(avgx2),int(avgy2)),[255,255,255],12) #draw left line
        avgx1,avgy1,avgx2,avgy2=avgRight
        cv2.line(img, (int(avgx1), int(avgy1)),(int(avgx2),int(avgy2)),[255,255,255],12) #draw right line
        return


    for line in lines:
        for x1,y1,x2,y2 in line:
            size = math.hypot(x2 - x1, y2 - y1)
            slope = ((float(y2)-y1)/(float(x2)-x1))

            # Filter slope based on incline and
            # find the most dominent segment based on length
            if (slope > 0.5): #right
                if (size > largestRightLineSize):
                    largestRightLine = (x1,y1,x2,y2)
                cv2.line(img,(x1,y1),(x2, y2),color,1)
            elif (slope < -0.5): #left
                if (size > largestLeftLineSize):
                    largestLeftLine = (x1,y1,x2,y2)
                cv2.line(img,(x1,y1),(x2,y2),color,1)

    # Define an imaginary horizontal line in the center of the screen
    # and at the bottom of the image, to extrapolate determined segment
    imgHeight,imgWidth = img.shape
    upLinePoint1 = np.array( [0, int((2.0/3.0)*imgHeight)] )
    upLinePoint2 = np.array( [imgWidth,int((2.0/3.0)*imgHeight)] )
    downLinePoint1 = np.array([0,imgHeight])
    downLinePoint2 = np.array([imgWidth,imgHeight])

    # Find the intersection of dominant lane with an imaginary horizontal line
    # in the middle of the image and at the bottom of the image.
    p3 = np.array( [largestLeftLine[0], largestLeftLine[1]] )
    p4 = np.array( [largestLeftLine[2], largestLeftLine[3]] )
    
    upLeftPoint = seg_intersect(upLinePoint1,upLinePoint2, p3,p4)
    downLeftPoint = seg_intersect(downLinePoint1,downLinePoint2, p3,p4)

    if (np.isinf(upLeftPoint[0]) or np.isinf(downLeftPoint[0])):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
        return
    # cv2.line(img, (int(upLeftPoint[0]), int(upLeftPoint[1])), (int(downLeftPoint[0]), int(downLeftPoint[1])), [0, 0, 255], 8) #draw left line

    # Calculate the average position of detected left lane over multiple video frames and draw
    avgx1,avgy1,avgx2,avgy2 = avgLeft
    avgLeft=(movingAverage(avgx1,upLeftPoint[0]),movingAverage(avgy1,upLeftPoint[1]),movingAverage(avgx2,downLeftPoint[0]),movingAverage(avgy2,downLeftPoint[1]))
    avgx1,avgy1,avgx2,avgy2 = avgLeft
    cv2.line(img,(int(avgx1),int(avgy1)),(int(avgx2),int(avgy2)),[255,255,255],12) #draw left line

    # Find the intersection of dominant lane with an imaginary horizontal line
    # in the middle of the image and at the bottom of the image.
    p5 = np.array( [largestRightLine[0], largestRightLine[1]] )
    p6 = np.array( [largestRightLine[2], largestRightLine[3]] )
    upRightPoint = seg_intersect(upLinePoint1,upLinePoint2,p5,p6)
    downRightPoint = seg_intersect(downLinePoint1,downLinePoint2,p5,p6)
    if (np.isinf(upRightPoint[0]) or np.isinf(downRightPoint[0])):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        # cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        # cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
        return
    # cv2.line(img, (int(upRightPoint[0]), int(upRightPoint[1])), (int(downRightPoint[0]), int(downRightPoint[1])), [0, 0, 255], 8) #draw left line

    # Calculate the average position of detected right lane over multiple video frames and draw
    avgx1, avgy1, avgx2, avgy2 = avgRight
    avgRight = (movingAverage(avgx1, upRightPoint[0]), movingAverage(avgy1, upRightPoint[1]), movingAverage(avgx2, downRightPoint[0]), movingAverage(avgy2, downRightPoint[1]))
    avgx1, avgy1, avgx2, avgy2 = avgRight
    cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line

def region_of_interest(img,vertices):
    vertices = np.array([vertices],dtype=np.int32)
    mask = np.zeros(img.shape,np.uint8)
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,)*channel_count

    else:
        ignore_mask_color = 255
    cv2.fillPoly(mask,vertices,ignore_mask_color)
    masked_image = cv2.bitwise_and(img,mask)
    return masked_image

def hough_lines(img,srcImg,rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros(img.shape, dtype=np.uint8)
    draw_lines(line_img,srcImg,lines)
    return line_img


################################################################################################################################


#read image
img = cv2.imread('lane.jpeg')
#apply blur
blurImg = cv2.GaussianBlur(img,(11,11),0) #kernel size 11x11
#canny edge detection
edgesImage = cv2.Canny(blurImg,40,50)

#dimensions of image
imgHeight,imgWidth,_ = img.shape
# vertices = np.array([[[3*imgWidth/4,3*imgHeight/5],[imgWidth/4,3*imgHeight/5],[40,imgHeight],[imgWidth-40,imgHeight]]],dtype=np.int32)
vertices = [[3*imgWidth/4,3*imgHeight/5],[imgWidth/4,3*imgHeight/5],[40,imgHeight],[imgWidth-40,imgHeight]]

roi = region_of_interest(edgesImage,vertices)
lineMarkedImage = hough_lines(roi,img,1, np.pi/180, 40, 30, 200)


cv2.imshow('roi',roi)
cv2.waitKey(0)

cv2.imshow('hough lines',lineMarkedImage)
cv2.waitKey(0)
