# 空间分割法及KLAMPT例程

-------------------------------------------

## KLAMPT环境部署
* 参考[krishauser的github](krishauser)
* 安装
    - 安装python3
    - >> pip install klampt
    - >> pip install PyOpenGL
    - >> pip install PyQt5
    -
* 环境测试
>> git clone http://github.com/krishauser/Klampt-examples
>> cd Klampt-examples/Python/demos
>> python gl_vis.py


* 例程代码下载
>> https://github.com/whutddk/Klampt_Robotic_Arm_Script.git


------------------------

## 路径栅格编码
此部分相当消耗运算资源，建议在服务器中进行运行，更建议充分优化代码效率后使用

### 立方栅格模型脚本
* 将demo/gridModelEncode/CartesianCoordinates/CaMain.py文件中的加载，保存文件地址设置正确
    - make_testing_mesh()中，**grid.loadFile("../../../terrains/cube.off")**指向标准正方体模型文件
    - load_Pose()中，**with open('../../result/create_Edge_3m250ms/jointList.json','r')**指向静态姿态列表json文件
    - load_edge()中，**with open('F:/klampt/250msx3grid/edge.json','r')**指向需要编码的路径列表json文件
    - load_Index()中，**with open('../../result/create_Edge_3m250ms/HeatCut/4096/edgeIndex.json','r')**指向路径索引json文件
    - store_Edge()中，**with open('F:/klampt/250msx3grid/edge.json','w')**指向需要保存的栅格数据的地址
    - __main__中，**res = world.readFile('../../../anno_check.xml')**指向世界模型文件，主要控制机械臂和场景位置
* 根据需要修改部分代码
    - __main__中，**  vis.add("world",world)**和**vis.show()**负责可视化，当运行于服务器等不可视环境时，和批量加速运行时，需要关闭该选项
* >> python CaMain.py



### 平面极坐标栅格模型脚本
* 将demo/gridModelEncode\PolarCoordinates/poMain.py文件中的加载，保存文件地址设置正确
    - make_testing_mesh()中，**grid.loadFile("../../../terrains/cube.off")**指向标准正方体模型文件
    - load_Pose()中，**with open('../../result/create_Edge_3m250ms/jointList.json','r')**指向静态姿态列表json文件
    - load_edge()中，**with open('F:/klampt/250msx3grid/edge.json','r')**指向需要编码的路径列表json文件
    - load_Index()中，**with open('../../result/create_Edge_3m250ms/HeatCut/4096/edgeIndex.json','r')**指向路径索引json文件
    - store_Edge()中，**with open('F:/klampt/250msx3grid/edge.json','w')**指向需要保存的栅格数据的地址
    - __main__中，**res = world.readFile('../../../anno_check.xml')**指向世界模型文件，主要控制机械臂和场景位置
* 根据需要修改部分代码
    - __main__中，**  vis.add("world",world)**和**vis.show()**负责可视化，当运行于服务器等不可视环境时，和批量加速运行时，需要关闭该选项


* 因为平面极坐标模型各个栅格大小不一致，首先需要通过脚本生成栅格模型
>> cd demo/gridModelEncode/PolarCoordinates/polarModel
>> python createPolar.py
* 接着执行主函数
>> python poMain.py



### 球面坐标栅格模型脚本
* 将demo/gridModelEncode\PolarCoordinates/poMain.py文件中的加载，保存文件地址设置正确
    - make_testing_mesh()中，**grid.loadFile("../../../terrains/cube.off")**指向标准正方体模型文件
    - load_Pose()中，**with open('../../result/create_Edge_3m250ms/jointList.json','r')**指向静态姿态列表json文件
    - load_edge()中，**with open('F:/klampt/250msx3grid/edge.json','r')**指向需要编码的路径列表json文件
    - load_Index()中，**with open('../../result/create_Edge_3m250ms/HeatCut/4096/edgeIndex.json','r')**指向路径索引json文件
    - store_Edge()中，**with open('F:/klampt/250msx3grid/edge.json','w')**指向需要保存的栅格数据的地址
    - __main__中，**res = world.readFile('../../../anno_check.xml')**指向世界模型文件，主要控制机械臂和场景位置
* 根据需要修改部分代码
    - __main__中，**  vis.add("world",world)**和**vis.show()**负责可视化，当运行于服务器等不可视环境时，和批量加速运行时，需要关闭该选项


* 因为球面坐标模型各个栅格大小不一致，首先需要通过脚本生成栅格模型
>> cd demo/gridModelEncode/sphericalCoordinate/spModel
>> python createTrapezoid.py
* 接着执行主函数
>> python spMain.py



---------------------

## 碰撞数据生成代码文件


* 生成真值表 *demo/createTrueTable/TrueTable.py*
    - 修改load_CollideData(num)中的，**with open('../offLineCollisionChecking/gridEncode'+ str(num) +'.txt','r')**指向路径栅格生成数据文件
    - 修改createTrueTable(num)中的**with open('./trueTable'+ str(num) +'.json','w')**指向生成真值表的目的地址
    - 执行脚本
    - >> python TrueTable.py


* 生成C查找表，*demo/truetable-2-Carray/createCmemory.py*
    - 修改write_Cfile()中，**with open('./CMemoryTable.c','w')**指向生成c查找表的目标地址
    - 修改load_trueTable()中，**with open( '../../../Klampt_Robotic_Arm_Script/state2/createTrueTable/trueTable'+ str(k) +'.json','r')**指向真值表文件地址
    - 执行脚本
    - >> python createCmemory.py

* 生成verilog表，*demo/truetable-2-Verilog/createLUTX1Verilog.py*
    - 修改load_trueTable(k)中，**with open( '../../../Klampt_Robotic_Arm_Script/state2/createTrueTable/trueTable'+ str(k) +'.json','r')**指向真值表文件地址
    - 修改write_verilog(k)，**with open('./trueTableP'+ str(k) +'.v','w') as verilogFile**指向生成verilog文件地址
    - 执行脚本
    - >>python createLUTX1Verilog.py
    - 配合顶层文件工作，顶层文件位于*demo/collide_detect*中








