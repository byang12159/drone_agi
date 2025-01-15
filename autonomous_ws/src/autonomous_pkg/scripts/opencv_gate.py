import cv2
import numpy as np
import csv
import os

real_radius_79 = 78  
real_radius_59 = 58
scaling_ratio = real_radius_79 / real_radius_59

tracker = cv2.TrackerCSRT_create() 

def process_frame_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 100, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    red_img = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (9, 9), 2), 50, 150)
    return edges

def detect_ellipses_in_contours(contours, min_area=100):
    ellipses = []
    for contour in contours:
        if len(contour) >= 5 and cv2.contourArea(contour) >= min_area:
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse
            if 0.7 <= MA / ma <= 1.3 and 160 <= MA <= 300 and 120 <= ma <= 300:
                ellipses.append((ellipse, cv2.contourArea(contour)))
    return ellipses

def select_best_double_ellipses(ellipses):
    if not ellipses:
        return [], []
    sorted_ellipses = sorted(ellipses, key=lambda e: e[1], reverse=True)
    best_ellipses = sorted_ellipses[:2]
    ellipse1, _ = best_ellipses[0]
    current_radius1 = (ellipse1[1][0] + ellipse1[1][1]) / 4
    if len(best_ellipses) > 1:
        ellipse2, _ = best_ellipses[1]
        current_radius2 = (ellipse2[1][0] + ellipse2[1][1]) / 4
    else:
        ellipse2 = ellipse1
        current_radius2 = current_radius1
    return [ellipse1, ellipse2], [current_radius1, current_radius2]

def calculate_depth(bbox_height):
    H_real = 0.8  # Gate height in meters
    focal_length = 547.57957461
    return (focal_length * H_real) / bbox_height

init = False

def process_frame(frame):
    global init
    gate_pos = []
    if not init:
        edges_red = process_frame_red(frame)
        contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ellipses_red = detect_ellipses_in_contours(contours_red, min_area=25)
        ellipses_pair, radius_pair = select_best_double_ellipses(ellipses_red)
        
        if len(ellipses_pair) > 0:
            ellipse = ellipses_pair[0]
            ellipse_points_1 = cv2.ellipse2Poly(
                center=(int(ellipse[0][0]), int(ellipse[0][1])),
                axes=(int(ellipse[1][0] / 2), int(ellipse[1][1] / 2)),
                angle=int(ellipse[2]),
                arcStart=0,
                arcEnd=360,
                delta=1
            )
            x_min_1, y_min_1, width_rect_1, height_rect_1 = cv2.boundingRect(ellipse_points_1)
            bbox = (x_min_1, y_min_1, width_rect_1, height_rect_1)
            tracker.init(frame, bbox)
            init = True
            depth_1 = calculate_depth(height_rect_1)
            gate_pos.append(x_min_1 + width_rect_1 / 2 - 320)
            gate_pos.append(y_min_1 + height_rect_1 / 2 - 240)
            gate_pos.append(depth_1)
    
    else:
        success, bbox = tracker.update(frame)
        if success:
            x_min, y_min, width_rect, height_rect = [int(v) for v in bbox]
            cv2.rectangle(frame, (x_min, y_min), (x_min + width_rect, y_min + height_rect), (0, 255, 0), 2)
            depth = calculate_depth(height_rect)
            gate_pos.append(x_min + width_rect / 2 - 320)
            gate_pos.append(y_min + height_rect / 2 - 240)
            gate_pos.append(depth)
        else:
            init = False

    return frame, gate_pos

def process_video(video_path, output_video_path):
    cap = cv2.VideoCapture(video_path)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_video_path, fourcc, 30.0, (int(cap.get(3)), int(cap.get(4))))
    
    initialized = False
    x_list, y_list, z_list = [], [], []
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if not initialized:
            edges_red = process_frame_red(frame)
            contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            ellipses_red = detect_ellipses_in_contours(contours_red, min_area=25)
            ellipses_pair, radius_pair = select_best_double_ellipses(ellipses_red)
            
            if len(ellipses_pair) > 0:
                ellipse = ellipses_pair[0]
                ellipse_points_1 = cv2.ellipse2Poly(
                    center=(int(ellipse[0][0]), int(ellipse[0][1])),
                    axes=(int(ellipse[1][0] / 2), int(ellipse[1][1] / 2)),
                    angle=int(ellipse[2]),
                    arcStart=0,
                    arcEnd=360,
                    delta=1
                )
                x_min_1, y_min_1, width_rect_1, height_rect_1 = cv2.boundingRect(ellipse_points_1)
                bbox = (x_min_1, y_min_1, width_rect_1, height_rect_1)
                tracker.init(frame, bbox)
                initialized = True
                depth_1 = calculate_depth(height_rect_1)
                x_list.append(x_min_1 + width_rect_1 / 2)
                y_list.append(y_min_1 + height_rect_1 / 2)
                z_list.append(depth_1)
        
        else:
            success, bbox = tracker.update(frame)
            if success:
                x_min, y_min, width_rect, height_rect = [int(v) for v in bbox]
                cv2.rectangle(frame, (x_min, y_min), (x_min + width_rect, y_min + height_rect), (0, 255, 0), 2)
                depth = calculate_depth(height_rect)
                x_list.append(x_min + width_rect / 2)
                y_list.append(y_min + height_rect / 2)
                z_list.append(depth)
            else:
                initialized = False
        
        out.write(frame)
        cv2.imshow('Video Output', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

    csv_file_path = "output_xyz.csv"
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])
        for x, y, z in zip(x_list, y_list, z_list):
            writer.writerow([x, y, z])
    
    print(f"Data written to {csv_file_path}")

# video_path = 'gate.mp4'  
# output_video_path = 'output_video.mp4'  
# process_video(video_path, output_video_path)