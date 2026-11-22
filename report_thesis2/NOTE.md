DISABLE_WAYLAND=1 GTK_IM_MODULE=xim code



Sau khi clone, trước tiên chọn integration tương ứng:
cd ~/ros2_ws
uv tool install specify-cli==1.4.3

# Chọn một:
specify integration switch claude
specify integration switch grok
specify integration switch agy
specify integration switch codex
Sau đó gửi AI prompt đầu tiên này:
Hãy tiếp quản repository hiện tại theo trạng thái đã lưu trong Git.

Ở bước đầu, chưa sửa code, chưa commit, chưa push và không đọc toàn bộ source.

Hãy thực hiện theo thứ tự:

1. Chạy `./uavcup context`.
2. Đọc đầy đủ:
   - `AGENTS.md`
   - `.specify/memory/constitution.md`
   - `docs/PROJECT_STATUS.md`
   - toàn bộ artifact của active feature trong `specs/`
   - `docs/ARCHITECTURE.md`
   - `docs/ROS2_CONTRACTS.md`
3. Nếu công việc liên quan Raspberry Pi, đọc thêm `docs/RASPI_SYNC.md`.
4. Kiểm tra branch, `git status`, remote và submodule; không reset hoặc làm mất thay đổi hiện có.
5. Chỉ đọc những source/config/test được active plan và task hiện tại chỉ ra.

Sau đó hãy báo cáo ngắn gọn cho tôi:
- active feature và trạng thái hiện tại;
- task đã hoàn thành;
- task chưa hoàn thành đầu tiên;
- package/file dự kiến cần đọc hoặc sửa;
- safety constraints liên quan;
- kết quả kiểm chứng gần nhất;
- blocker hoặc bước đang `NOT RUN`;
- hành động tiếp theo bạn đề xuất.

Không được tự suy đoán rằng test, SSH, đồng bộ Raspberry Pi hoặc flight test đã pass nếu không có bằng chứng trong `verification.md`.
Hãy dừng sau báo cáo để tôi xác nhận trước khi triển khai.
Sau khi AI báo cáo đúng, prompt thứ hai chỉ cần:
Đồng ý. Hãy tiếp tục task chưa hoàn thành đầu tiên theo plan.

Trước khi sửa, đọc source/config/test liên quan. Sau khi làm xong:
- chạy verification phù hợp;
- cập nhật `tasks.md`;
- cập nhật `verification.md`;
- cập nhật `docs/PROJECT_STATUS.md`;
- báo rõ PASS, FAIL hoặc NOT RUN;
- chưa commit hoặc push nếu tôi chưa yêu cầu.
Nếu muốn AI tự làm tiếp ngay không cần chờ xác nhận, thay câu cuối của prompt đầu bằng:
Nếu không có blocker hoặc mâu thuẫn tài liệu, hãy tiếp tục task chưa hoàn thành đầu tiên, chạy verification và cập nhật toàn bộ artifact bàn giao trước khi kết thúc.

    
10:11 AM