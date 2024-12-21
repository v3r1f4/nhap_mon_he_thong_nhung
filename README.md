THIẾT KẾ VÀ THỰC HIỆN HỆ THỐNG PHÁT PHÁT HIỆN VÀ ĐO KHOẢNG CÁCH ĐẾN VẬT CẢN

1. Giới thiệu (Introduction)
Mục tiêu của dự án: Vận dụng các kiến thức, kỹ năng đã được học để thiết kế và thực thi một hệ thống nhúng phát hiện và đo khoảng cách tới vật cản ứng dụng cho các xe tự hành. Hệ thống được thiết kế để có thể phát hiện vật cản và đo khoảng cách đến vật thể bằng bo mạch STM32 Nucleo-F401RE và các mô-đun cảm biến như cảm biến siêu âm, cảm biến hồng ngoại, radar. Sinh viên cũng có thể sử dụng một bo mạch tương đương (dùng vi xử lý ARM) để thực hiện dự án.

4. Yêu cầu đối với thiết kế (Requirements)
Yêu cầu thiết kế:
Chức năng: phát hiện người vật cản và đo khoảng cách đến vật cản và hiển thị kết quả trên màn LCD.
CPU: vi điều khiển sử dụng vi xử lý ARM-Cortex-M (ví dụ STM32 Nucleo-F401re
Các đầu vào:
Hệ thống sử dụng cảm biến siêu âm, hồng ngoại hoặc radar để phát hiện và đo khoảng cách đến vật cản.
SW1 để chuyển đổi hệ thống giữa trạng thái hoạt động và trạng thái dừng. Khi hệ thống dừng hoạt động, nếu bấm SW1 bộ đếm chuyển sang trạng thái hoạt động, và ngược lại. Khi chuyển từ trạng thái dừng sang trạng thái hoạt động, hệ thống tiếp tục giám sát trạng thái của cảm biến đo khoảng cách và hiển thị trên LCD.
SW2 để xóa (reset) trạng thái hệ thống về trạng thái ban đầu.
Các đầu ra:
Hệ thống có 2 lối ra trạng thái: LED xanh nhấp nháy với tần số 1Hz khi hệ thống hoạt động; tắt khi hệ thống dừng hoạt động. LED đỏ nháy ở tần số 2Hz-10Hz khi hệ thống phát hiện vật cản – tần số nháy càng cao khi vật cản ở càng gần hệ đo; tắt khi hệ thống ở trạng thái bình thường.
LCD:
Hiện thị trạng thái của hệ thống: 0 – bình thường, 1 – hoạt động
Trạng thái có vật cản hay không: 0 – không có, 1 – có vật cản
Khoảng cách đến vật cản khi phát hiện có vật cản
Sử dụng các timer để xác định khoảng cách tới vật cản và thời gian nhấp nháy các LED.
Khuyến khích SV lắp ráp một xe tự hành tích hợp hệ thống nhúng trên (sẽ được cộng điểm)
