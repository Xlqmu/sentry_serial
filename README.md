现在的思路就是模式切换+定点
但是现在卡在模式切换的逻辑上，定点的逻辑似乎没有什么问题
主要逻辑是通过从下位机处理的topic数据来判断是否需要去执行什么任务，
如一但从topic相关数据中读到比赛状态为4时开始导航去中心点，通过从topic读到的数据时刻检测自身血量，一旦残血就去起始点回血，然后血回满就去中心点，到达中心点(导航任务完成后)就开启自瞄模式（发给下位机的数据包中的move_mode设为2），其余导航时设置为4

# 区域赛串口简陋版

初步解耦，编译通过，尚待测试

放弃模式切换，换为确定的模式，转速交由电控端控制