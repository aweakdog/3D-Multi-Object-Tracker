

1.使用

依赖:
private_msgs(这个是定位gnss 临时话题,之后可能替换掉)
udi_msgs

运行:

a.将3D-MULTI-OBJECT-TRACKER,private_msgs,udi_msgs放到workspace/src下
b.在workspace下catkin_make
c.python3 ./src/3D-Multi-Object-Tracker/udi_castrack.py (默认使用8个核实际上只需要用1个核来满足需求,所以需要特别指定)
单一cpu: taskset -c 0 python3 ./src/3D-Multi-Object-Tracker/udi_castrack.py

2.输入输出话题
输入:
/perception/detOBB  #检测结果
/gnss_coords #private_msgs定位结果
/localization_estimate #udim_msgs的定位结果

输出:

/prediction_obstacles #tracking结果
/bounding_boxes #可视化结果

3.可以调的参数

在udi_castrack.py 的Config里
参数含义参考原论文CasTrack 
https://ieeexplore.ieee.org/abstract/document/9352500

和原论文比本项目只修改了仓库内的udi_castrack.py里的内容，调用原论文的仓库的函数







