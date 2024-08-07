# 曲率计算
![image](https://github.com/user-attachments/assets/0ed3c884-ef34-4948-8e55-f71b306de4a9)

# lqr 推导
![image](https://github.com/user-attachments/assets/951b7a1c-5401-4211-a0dc-34776080d6b2)
![image](https://github.com/user-attachments/assets/5a936b0a-248c-450a-99f8-ac9ca944cd13)


# 迭代法求黎卡提（Riccati）方程的解
![image](https://github.com/user-attachments/assets/a671411d-ee5e-40ae-b224-d94c746d2a47)


# command
```shell
mkdir build
cd build/
cmake ..
make
``` 

## 运行
``` shell
cd build/
./lqr_controller    //lqr
./lqr_pid_controller  //lqr + pid
```
## 结果
### LQR   (恒速 2m/s)
![image](https://github.com/user-attachments/assets/9365fa22-cf6b-432c-b75e-4718139d3e1c)
### LQR + PID  （初始速度为0 ， 目标速度为4m/s）
![image](https://github.com/user-attachments/assets/f3d510fe-dae4-45bb-865e-7d8781b47088)
