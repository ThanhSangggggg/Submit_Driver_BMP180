# Submit_Driver_BMP180

# Thành viên và MSSV
1.Nguyễn Thanh Sang - 22146388
2.Phạm Thiên Phú    - 22146370
3.Cao Ngọc Quí      - 22146387
4.Nguyễn Võ Bảo An  - 22146261

# Các lưu ý để vận hành Driver BMP180
- Đầu tiên: Tải tất cả các file vào chung một Folder
- Thứ hai : chạy dòng lệnh "sudo rmmod bmp280-i2c" (cho chắc: để đảm bảo không ưu tiên file Driver BMP280 tồn tại trong hệ điều hành Linux so với BMP180 vừa tạo).
- Tiếp theo: chạy lệnh "make" cho ra kết quả Nhiệt độ (Temperature), Áp suất (Pressure) và Độ cao tương đối (Altitude) mười lần mỗi giây. (Các lệnh chạy Driver đã tích hợp vào Makefile).
  *Note: Muốn xem nhiều lần thì chạy lệnh "sudo ./run" nhiều hoặc dùng "make clean" xong "make" lại.
- Kết thúc: Chạy lệnh "make clean"  để xóa toàn bộ dữ liệu đã tạo, trở về ban đầu như chưa cài Driver.
