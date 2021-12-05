# Task 1

### Question

补充直接线性方法的里程计标定模块代码

### Answer

```c++
        // TODO
        tf::Quaternion q_start = frame_start_pose.getRotation();
        tf::Quaternion q_end = frame_end_pose.getRotation();
        tf::Vector3 t_start(frame_start_pose.getOrigin().getX(), frame_start_pose.getOrigin().getY(), 1);
        tf::Vector3 t_end(frame_end_pose.getOrigin().getX(), frame_end_pose.getOrigin().getY(), 1);

        for (int i = startIndex; i < startIndex + beam_number; ++i)
        {
            tf::Vector3 mid_xy = t_start.lerp(t_end, (i - startIndex) / (beam_number - 1));
            tf::Quaternion mid_q = q_start.slerp(q_end, (i - startIndex) / (beam_number - 1));
            tf::Transform mid_frame(mid_q, mid_xy);
            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);
            tf::Vector3 calib_point = frame_base_pose.inverse() * mid_frame * tf::Vector3(x, y, 1);
            ranges[i] = sqrt(calib_point[0] * calib_point[0] + calib_point[1] * calib_point[1]);
            angles[i] = atan2(calib_point[1], calib_point[0]);
        }
        // end of TODO
```


### Results

![task1](./docs/screenshot.png)

# Task 2

### Question

补充基于模型方法的里程计标定模块代码

### Answer


见pdf [here](./docs/Task2.pdf)

# Task 3

### Question

1. 对于该类问题，你都知道哪几种求解方法？
2. 各方法的优缺点有哪些？分别在什么条件下较常被使用？

### Answer


见pdf [here](./docs/Task3.pdf)

# Task 4

### Question

设计里程计与激光雷达外参标定方法

### Answer

见pdf [here](./docs/Task4.pdf)