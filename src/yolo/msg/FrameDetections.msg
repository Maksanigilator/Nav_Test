# Результат обработки одного кадра MJPEG-потока.
#
# Одно сообщение = один кадр, чтобы потребителю не нужно было сопоставлять
# детекции и позу по времени вручную:
#   header.stamp — ROS-время ПОЛУЧЕНИЯ кадра (на него же взята поза);
#   detections   — рамки в пикселях кадра (frame_id сверху);
#   robot_pose   — поза робота ровно на время кадра.

std_msgs/Header header

# Найденные объекты. У каждой Detection2D.header.stamp тот же, что сверху.
vision_msgs/Detection2D[] detections

# Поза робота в момент кадра; header.stamp у неё тоже совпадает.
# Осмысленна только при pose_valid == true.
geometry_msgs/PoseWithCovarianceStamped robot_pose

# false — позу получить не удалось (нет TF, топик молчит или данные устарели).
bool pose_valid
