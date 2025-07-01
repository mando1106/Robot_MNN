# 🚀 这是一个pytorch 神经网络部署到  ARMv7 32位的示例项目

# 步骤
> 第一步 使用onxx 在pytorch中将模型导出文件 （需要pip install onnx）

> 第二步下载 MNN 第三方源码 ，在该文件夹下，建立两个build文件，一个用于编译使用与本机的库（一般是x86-64）.一个库是用来进行交叉编译（在这之前需要下载一个适配ARMv7 的交叉编译链）


> 第三步 使用本机 make install 编译得到的应用程序 MNNconventor 对导出的文件进行转换，类似：
/home/mando/Phd/Ec_mater/mando_torch/MNN/build/MNNConvert -f ONNX --modelFile your_model.onnx --MNNModel your_model.mnn --bizCode biz

>第四步，可以编译使用于本机或者ARMv7的应用程序，详情见两个分支

---



## 📦 功能特性（Features）
- ✨ 支持 ARMv7 MNN 模型推理
- ⚙️ 支持交叉编译部署
- 📈 实时数据可视化

---

## 🔧 安装与使用（Installation & Usage）

```bash
# 克隆仓库
https://github.com/mando1106/Robot_MNN.git
cd Robot_MNN

将我编译得到的main文件和PISMP_model.mnn 复制到对应的架构中应该能运行
./main
