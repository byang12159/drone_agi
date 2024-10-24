import cv2
import numpy as np
import os
import pandas as pd


real_radius_79 = 78  
real_radius_59 = 58
scaling_ratio = real_radius_79 / real_radius_59

def process_frame_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 100, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    red_img = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Grayscale Image', gray)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (9, 9), 2), 50, 150)
    # cv2.imshow('Edges', edges)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return edges

def process_frame_black(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 70])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    non_black_mask = cv2.bitwise_not(mask)
    black_masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
    black_masked_frame[non_black_mask == 255] = [255, 255, 255]
    return black_masked_frame

def detect_ellipses_in_contours(contours, min_area=100):
    ellipses = []
    for contour in contours:
        if len(contour) >= 5 and cv2.contourArea(contour) >= min_area:
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse
            if 0.7 <= MA / ma <= 1.3 and 160 <= MA <= 300 and 120 <= ma <= 300:
                ellipses.append((ellipse, cv2.contourArea(contour)))
    return ellipses

def select_best_ellipse(ellipses, previous_inside, previous_radius):
    if not ellipses:
        return None, None, previous_inside, previous_radius
    inside = False
    ellipse, _ = max(ellipses, key=lambda e: e[1])
    (x, y), (MA, ma), _ = ellipse
    current_radius = (MA + ma) / 4
    if previous_radius and current_radius < previous_radius * 0.85:
        real_radius = real_radius_59 * scaling_ratio
        inside = True
    else:
        real_radius = real_radius_79
    previous_radius = current_radius
    return ellipse, real_radius, inside, previous_radius

def select_best_double_ellipses(ellipses):
    if not ellipses:
        return [], []
    
    inside = False
    
    # Sort ellipses by area in descending order and select the top 2
    sorted_ellipses = sorted(ellipses, key=lambda e: e[1], reverse=True)
    best_ellipses = sorted_ellipses[:2]
    
    # Extract parameters for the first ellipse
    ellipse1, _ = best_ellipses[0]
    (x1, y1), (MA1, ma1), _ = ellipse1
    current_radius1 = (MA1 + ma1) / 4
    
    # Extract parameters for the second ellipse
    if len(best_ellipses) > 1:
        ellipse2, _ = best_ellipses[1]
        (x2, y2), (MA2, ma2), _ = ellipse2
        current_radius2 = (MA2 + ma2) / 4
    else:
        ellipse2 = ellipse1
        current_radius2 = current_radius1
    
    return [ellipse1, ellipse2], [current_radius1, current_radius2]

# Depth estimation constants
H_real = 0.8  # Gate height in meters
focal_length = 547.57957461


# Function to calculate depth based on bounding box height
def calculate_depth(bbox_height):
    return (focal_length * H_real) / bbox_height

def detect_gates_in_images(input_folder, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    previous_radius = None
    inside = False

    for filename in os.listdir(input_folder):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            image_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)

            frame = cv2.imread(image_path)
            if frame is None:
                continue

            edges_red = process_frame_red(frame)
            # black_masked_frame = process_frame_black(frame)
            # gray_black = cv2.cvtColor(black_masked_frame, cv2.COLOR_BGR2GRAY)
            # edges_black = cv2.Canny(cv2.GaussianBlur(gray_black, (9, 9), 2), 110, 240)

            contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # contours_black, _ = cv2.findContours(edges_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # print(contours_red)
            ellipses_red = detect_ellipses_in_contours(contours_red)
            # ellipses_black = detect_ellipses_in_contours(contours_black, min_area=200)
            # print(ellipses_red)
            ellipses = ellipses_red
            # ellipse, real_radius, inside, previous_radius = select_best_ellipse(ellipses, inside, previous_radius)
            ellipses_pair, radius_pair = select_best_double_ellipses(ellipses)
            if len(ellipses_pair) > 0:
                if radius_pair[0] > radius_pair[1]:
                    ellipse = ellipses_pair[0]
                    ellipse_2 = ellipses_pair[1]
                else:
                    ellipse = ellipses_pair[1]
                    ellipse_2 = ellipses_pair[0]

                if ellipse:
                    ellipse_points_1 = cv2.ellipse2Poly(
                        center=(int(ellipse[0][0]), int(ellipse[0][1])),
                        axes=(int(ellipse[1][0] / 2), int(ellipse[1][1] / 2)),
                        angle=int(ellipse[2]),
                        arcStart=0,
                        arcEnd=360,
                        delta=1
                    )
                    x_min_1, y_min_1, width_rect_1, height_rect_1 = cv2.boundingRect(ellipse_points_1)

                    cv2.rectangle(frame, (x_min_1, y_min_1), (x_min_1 + width_rect_1, y_min_1 + height_rect_1), (0, 255, 0), 2)
                    center_1 = (x_min_1+width_rect_1/2.0,y_min_1+height_rect_1/2.0)
                    depth_1 = calculate_depth(height_rect_1)


                    ellipse_points_2 = cv2.ellipse2Poly(
                        center=(int(ellipse_2[0][0]), int(ellipse_2[0][1])),
                        axes=(int(ellipse_2[1][0] / 2), int(ellipse_2[1][1] / 2)),
                        angle=int(ellipse_2[2]),
                        arcStart=0,
                        arcEnd=360,
                        delta=1
                    )
                    x_min_2, y_min_2, width_rect_2, height_rect_2 = cv2.boundingRect(ellipse_points_2)

                    cv2.rectangle(frame, (x_min_2, y_min_2), (x_min_2 + width_rect_2, y_min_2 + height_rect_2), (0, 255, 0), 2)
                    center_2 = (x_min_2+width_rect_2/2.0,y_min_2+height_rect_2/2.0)

                    center = ((center_1[0]+center_2[0])/2.0,(center_1[1]+center_2[1])/2.0)
                    if ellipse == ellipse_2:
                        depth = calculate_depth(height_rect_1)
                    else:
                        depth = (calculate_depth(height_rect_1) + calculate_depth(height_rect_2*scaling_ratio))/2
                    cv2.putText(frame, f"Gate (Center: {center[0]},{center[1]}, Depth: {depth}):", (x_min_1, y_min_1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


            cv2.imwrite(output_path, frame)

    print(f"Processed images saved to {output_folder}")

# Example usage:
input_folder = "pi_images_raw_circ"  # Replace with your input folder path
output_folder = "test"  # Replace with your output folder path
# detect_gates_in_images(input_folder, output_folder)
