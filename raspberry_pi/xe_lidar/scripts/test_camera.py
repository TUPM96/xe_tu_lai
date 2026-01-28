#!/usr/bin/env python3
"""
Script test Camera trên Raspberry Pi (không cần ROS2)
Sử dụng OpenCV để đọc và hiển thị ảnh từ camera USB
"""

import cv2
import sys
import argparse


def test_camera(device=0, width=1280, height=720):
    """
    Test camera với OpenCV
    
    Args:
        device: Device ID của camera (0, 1, 2...)
        width: Chiều rộng ảnh
        height: Chiều cao ảnh
    """
    print(f"🔍 Đang mở camera device /dev/video{device}...")
    print(f"   Kích thước: {width}x{height}")
    print("   Nhấn 'q' để thoát")
    print("   Nhấn 's' để lưu ảnh")
    print("-" * 50)
    
    # Mở camera
    cap = cv2.VideoCapture(device)
    
    if not cap.isOpened():
        print(f"❌ Không thể mở camera /dev/video{device}")
        print(f"   Kiểm tra:")
        print(f"   - Camera đã được kết nối chưa?")
        print(f"   - Quyền truy cập: sudo chmod 777 /dev/video{device}")
        print(f"   - Device ID đúng chưa? (ls /dev/video*)")
        return False
    
    # Đặt kích thước
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    # Kiểm tra kích thước thực tế
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera đã mở thành công!")
    print(f"   Kích thước thực tế: {actual_width}x{actual_height}")
    print("-" * 50)
    
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("❌ Không thể đọc frame từ camera")
                break
            
            frame_count += 1
            
            # Hiển thị thông tin
            cv2.putText(frame, f"Frame: {frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, "Press 'q' to quit, 's' to save", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Hiển thị ảnh
            cv2.imshow('Camera Test', frame)
            
            # Xử lý phím
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("👋 Đã thoát")
                break
            elif key == ord('s'):
                filename = f"camera_test_{frame_count}.jpg"
                cv2.imwrite(filename, frame)
                print(f"💾 Đã lưu ảnh: {filename}")
            
    except KeyboardInterrupt:
        print("\n👋 Đã dừng bởi người dùng")
    except Exception as e:
        print(f"❌ Lỗi: {str(e)}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"✅ Đã đóng camera. Tổng số frame: {frame_count}")
        return True


def main():
    parser = argparse.ArgumentParser(description='Test Camera trên Raspberry Pi')
    parser.add_argument('--device', type=int, default=0,
                       help='Device ID của camera (mặc định: 0)')
    parser.add_argument('--width', type=int, default=1280,
                       help='Chiều rộng ảnh (mặc định: 1280)')
    parser.add_argument('--height', type=int, default=720,
                       help='Chiều cao ảnh (mặc định: 720)')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("📷 TEST CAMERA - Raspberry Pi")
    print("=" * 50)
    
    success = test_camera(args.device, args.width, args.height)
    
    if success:
        print("=" * 50)
        print("✅ Camera hoạt động tốt!")
        print("=" * 50)
        sys.exit(0)
    else:
        print("=" * 50)
        print("❌ Camera không hoạt động")
        print("=" * 50)
        sys.exit(1)


if __name__ == '__main__':
    main()

