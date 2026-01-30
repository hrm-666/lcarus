## 简介

Perfei-fly项目是本人基于其他开源飞控制作，现已迭代到第五版，重构了整个框架，侧重于学习和开发，并增加了一部分功能。本项目旨在为飞控开发者提供一个入门级的学习模板，降低学习飞控的学习门槛。

## 说明

为便于管理，本仓仅存放飞机的源码，另增加了遥控器仓和文档仓。文档仓包含了使用说明、硬件、结构件等一切相关资料，想复刻的小伙伴请仔细浏览文档仓。另外旧版本的相关文件存放在文档仓的else文件夹下，需要的小伙伴请移至文档仓获取。

## 目录结构

```

├── cmsis		//存放Cortex微控制器软件接口标准文件。
├── driver 		//存放外设驱动，比如mpu6050、bmp280、ws2812驱动等。
├── module		 //各个功能模块，所有功能的.c文件都在里面。
├── my_lib 		//个人整理和封装的一些库，比如adc滤波、队列数据结构、log库等。
├── protocol	 	//外设协议层，比如i2c、spi、uart等。
├── rtthread		 //rtthread内核。
├── stm32_lib	 //stm32标准库。
├── system 		//存放跟单片机系统相关的接口，比如flash读写函数、滴答定时器等。
|── user 		//存放工程文件和main.c。
```

## 相关链接

成品购买链接：[链接](https://h5.m.goofish.com/item?forceFlush=1&id=994845221057&ut_sk=1.XoGoF00Rk0QDAA9C2X4dD3LZ_21407387_1763272562519.copy.detail.994845221057.2945000689)

文档仓：[https://gitee.com/gitee_Fei/Perfei-fly_doc](https://gitee.com/gitee_Fei/Perfei-fly_doc)

遥控器代码仓：[https://gitee.com/gitee_Fei/Perfei-fly_remote](https://gitee.com/gitee_Fei/Perfei-fly_remote)

飞机工程开源链接：[https://oshwhub.com/jialc_fei/perfei-flyboard_v5.0](https://oshwhub.com/jialc_fei/perfei-flyboard_v5.0)

遥控器工程开源链接：[https://oshwhub.com/jialc_fei/perfei-fly_remote_v5-0](https://oshwhub.com/jialc_fei/perfei-fly_remote_v5-0)

四轴无人机B站视频地址：[https://www.bilibili.com/video/BV1jkndz9EKh/](https://www.bilibili.com/video/BV1jkndz9EKh/)

飞机结构件链接：[https://makerworld.com.cn/zh/models/1624674-wu-ren-ji-ji-jia-8520kong-xin-bei#profileId-1778740](https://makerworld.com.cn/zh/models/1624674-wu-ren-ji-ji-jia-8520kong-xin-bei#profileId-1778740)

遥控器结构件链接：[https://makerworld.com.cn/zh/models/1624557-perfei-flyyao-kong-qi-jie-gou-jian#profileId-1778622](https://makerworld.com.cn/zh/models/1624557-perfei-flyyao-kong-qi-jie-gou-jian#profileId-1778622)
