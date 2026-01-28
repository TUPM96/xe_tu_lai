#!/usr/bin/env python3
"""
Script test Camera đầy đủ trên Raspberry Pi (không cần ROS2)
- Hiển thị ảnh real-time
- Test lane detection (phát hiện vạch kẻ đường)
- Lưu ảnh để kiểm tra
"""

import cv2
import numpy as np
import sys
import argparse
import time


def detect_lanes(image, roi_top=0.4):
    """
    Phát hiện vạch kẻ đường (giống như trong obstacle_avoidance.py)
    
    Args:
        image: Ảnh input (BGR)
        roi_top: Tỷ lệ phần trên để bỏ qua (0.4 = bỏ 40% trên)
    
    Returns:
        image_with_lanes: Ảnh đã vẽ vạch kẻ đường
        lane_center_offset: Offset từ giữa đường (-1 đến 1)
        lane_detected: True nếu phát hiện được vạch
    """
    height, width = image.shape[:2]
    
    # Tạo vùng quan tâm (ROI) - phần dưới ảnh
    roi_top_pixel = int(height * roi_top)
    roi = image[roi_top_pixel:height, :]
    
    # Chuyển sang HSV để dễ phát hiện màu trắng
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Tạo mask cho màu trắng (vạch kẻ đường)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Áp dụng Gaussian blur
    blurred = cv2.GaussianBlur(white_mask, (5, 5), 0)
    
    # Phát hiện cạnh bằng Canny
    edges = cv2.Canny(blurred, 50, 150)
    
    # Phát hiện đường thẳng bằng HoughLinesP
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, 
                           minLineLength=20, maxLineGap=15)
    
    # Tạo ảnh output
    image_with_lanes = image.copy()
    lane_center_offset = 0.0
    lane_detected = False
    
    if lines is not None and len(lines) > 0:
        # Phân loại đường thẳng thành bên trái và bên phải
        left_lines = []
        right_lines = []
        center_x = width / 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 != x1:
                slope = (y2 - y1) / (x2 - x1)
                mid_x = (x1 + x2) / 2
                
                # Đường bên trái: slope âm và nằm bên trái
                if slope < -0.2 and mid_x < center_x:
                    left_lines.append(line[0])
                # Đường bên phải: slope dương và nằm bên phải
                elif slope > 0.2 and mid_x > center_x:
                    right_lines.append(line[0])
                
                # Vẽ đường thẳng phát hiện được
                cv2.line(image_with_lanes, 
                        (x1, y1 + roi_top_pixel), 
                        (x2, y2 + roi_top_pixel), 
                        (0, 255, 0), 2)
        
        # Tính điểm trung bình
        left_x_points = []
        right_x_points = []
        
        for line in left_lines:
            x1, y1, x2, y2 = line
            if y1 > y2:
                left_x_points.append(x1)
            else:
                left_x_points.append(x2)
        
        for line in right_lines:
            x1, y1, x2, y2 = line
            if y1 > y2:
                right_x_points.append(x1)
            else:
                right_x_points.append(x2)
        
        # Tính offset từ giữa đường
        if left_x_points and right_x_points:
            left_x_avg = np.mean(left_x_points)
            right_x_avg = np.mean(right_x_points)
            lane_center = (left_x_avg + right_x_avg) / 2
            lane_center_offset = (lane_center - center_x) / (width / 2)
            lane_detected = True
            
            # Vẽ vạch trái/phải
            cv2.line(image_with_lanes, 
                    (int(left_x_avg), height), 
                    (int(left_x_avg), roi_top_pixel), 
                    (255, 0, 0), 3)
            cv2.line(image_with_lanes, 
                    (int(right_x_avg), height), 
                    (int(right_x_avg), roi_top_pixel), 
                    (0, 0, 255), 3)
            cv2.line(image_with_lanes, 
                    (int(lane_center), height), 
                    (int(lane_center), roi_top_pixel), 
                    (0, 255, 255), 2)
        elif left_x_points:
            left_x_avg = np.mean(left_x_points)
            lane_center = left_x_avg + 400  # HD resolution
            lane_center_offset = (lane_center - center_x) / (width / 2)
            lane_detected = True
            cv2.line(image_with_lanes, 
                    (int(left_x_avg), height), 
                    (int(left_x_avg), roi_top_pixel), 
                    (255, 0, 0), 3)
        elif right_x_points:
            right_x_avg = np.mean(right_x_points)
            lane_center = right_x_avg - 400  # HD resolution
            lane_center_offset = (lane_center - center_x) / (width / 2)
            lane_detected = True
            cv2.line(image_with_lanes, 
                    (int(right_x_avg), height), 
                    (int(right_x_avg), roi_top_pixel), 
                    (0, 0, 255), 3)
    
    # Vẽ ROI border
    cv2.line(image_with_lanes, 
            (0, roi_top_pixel), 
            (width, roi_top_pixel), 
            (255, 255, 0), 2)
    
    # Vẽ thông tin
    cv2.putText(image_with_lanes, f"Lane detected: {lane_detected}", 
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(image_with_lanes, f"Offset: {lane_center_offset:.2f}", 
               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return image_with_lanes, lane_center_offset, lane_detected


def test_camera_full(device=0, width=1280, height=720, enable_lane_detection=True):
    """
    Test camera đầy đủ với lane detection
    """
    print(f"🔍 Đang mở camera device /dev/video{device}...")
    print(f"   Kích thước: {width}x{height}")
    print(f"   Lane detection: {'Bật' if enable_lane_detection else 'Tắt'}")
    print("   Nhấn 'q' để thoát")
    print("   Nhấn 's' để lưu ảnh")
    print("   Nhấn 'd' để bật/tắt lane detection")
    print("-" * 50)
    
    cap = cv2.VideoCapture(device)
    
    if not cap.isOpened():
        print(f"❌ Không thể mở camera /dev/video{device}")
        return False
    
    # Dùng MJPG codec để hỗ trợ HD resolution
    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"✅ Camera đã mở!")
    print(f"   Kích thước thực tế: {actual_width}x{actual_height}")
    print(f"   FPS: {fps}")
    print("-" * 50)
    
    frame_count = 0
    fps_counter = 0
    fps_start_time = time.time()
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("❌ Không thể đọc frame")
                break
            
            frame_count += 1
            fps_counter += 1
            
            # Tính FPS
            current_time = time.time()
            if current_time - fps_start_time >= 1.0:
                actual_fps = fps_counter / (current_time - fps_start_time)
                fps_counter = 0
                fps_start_time = current_time
            else:
                actual_fps = 0
            
            # Lane detection nếu được bật
            if enable_lane_detection:
                processed_frame, offset, detected = detect_lanes(frame)
            else:
                processed_frame = frame.copy()
                offset = 0.0
                detected = False
            
            # Hiển thị thông tin
            info_text = [
                f"Frame: {frame_count}",
                f"FPS: {actual_fps:.1f}",
                f"Lane: {'YES' if detected else 'NO'}",
                f"Offset: {offset:.2f}"
            ]
            
            y_offset = 30
            for i, text in enumerate(info_text):
                cv2.putText(processed_frame, text, (10, y_offset + i * 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.putText(processed_frame, 
                       "Press 'q' to quit, 's' to save, 'd' to toggle detection",
                       (10, processed_frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Hiển thị
            cv2.imshow('Camera Test - Lane Detection', processed_frame)
            
            # Xử lý phím
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"camera_test_{frame_count}.jpg"
                cv2.imwrite(filename, processed_frame)
                print(f"💾 Đã lưu: {filename}")
            elif key == ord('d'):
                enable_lane_detection = not enable_lane_detection
                print(f"🔀 Lane detection: {'Bật' if enable_lane_detection else 'Tắt'}")
            
    except KeyboardInterrupt:
        print("\n👋 Đã dừng")
    except Exception as e:
        print(f"❌ Lỗi: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"✅ Đã đóng. Tổng frame: {frame_count}")
        return True


def main():
    parser = argparse.ArgumentParser(description='Test Camera đầy đủ với Lane Detection')
    parser.add_argument('--device', type=int, default=0,
                       help='Device ID camera (mặc định: 0)')
    parser.add_argument('--width', type=int, default=1280,
                       help='Chiều rộng (mặc định: 1280)')
    parser.add_argument('--height', type=int, default=720,
                       help='Chiều cao (mặc định: 720)')
    parser.add_argument('--no-lane', action='store_true',
                       help='Tắt lane detection')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("📷 TEST CAMERA ĐẦY ĐỦ - Raspberry Pi")
    print("=" * 50)
    
    success = test_camera_full(args.device, args.width, args.height, 
                              enable_lane_detection=not args.no_lane)
    
    if success:
        print("=" * 50)
        print("✅ Camera test hoàn thành!")
        print("=" * 50)
        sys.exit(0)
    else:
        print("=" * 50)
        print("❌ Camera test thất bại")
        print("=" * 50)
        sys.exit(1)


if __name__ == '__main__':
    main()

