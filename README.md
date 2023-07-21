# serial（linux）
**注意:由于这篇代码主要是解决单片机与nx版的问题，则以此为例子进行解释**  
## 连接前的准备
* 查看串口：在连接上单片机后可以查看/dev文件夹，会发现多了ttyACM0,ttyACM1,其中ttyACM1为连接上单片机的端口。  
* 赋予串口ttyACM1权限:`sudo chmod 666 /dev/ttyACM1`
* 安装serial:`sudo apt install ros-noetic-serial`
## 代码部分
示例代码如下：  
`serial::Serial ser;`  
`ser.setPort("/dev/ttyACM1");`  
`ser.setBaudrate(9600);`  
`serial::Timeout to = serial::Timeout::simpleTimeout(100);`  
`ser.setTimeout(to);`  
`ser.open();`  
之后利用ser.write和ser.read就可以进行串口通信了
