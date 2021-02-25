(21.02.24)
# [ROS]opencv_project1
노트북을 통해 특정 색상 (파란색)을 검출 한 후 터틀 심 거북이를 검출 된 위치로 이동시키는 프로젝트
## RQT_GRAPH
![rosgraph](https://user-images.githubusercontent.com/79581819/109004754-84f5e900-76ec-11eb-85fc-a05972cb8ff5.png)
## RESULT
![1](https://user-images.githubusercontent.com/79581819/109004988-cdada200-76ec-11eb-8bac-848983b45ef1.jpg)
![sad](https://user-images.githubusercontent.com/79581819/109004884-aa82f280-76ec-11eb-87f5-32f2adeb3fa9.jpg)

(21.02.25)
```
* 거북이와 목표지점 사이 angle이 angle > 180, anlge <-180가 되어 범위값을 초과하는 에러 발생
  : anlgular_detection함수 추가 -360도를 더하거나 빼줌으로 써 범위 값에 맞는 anlge로 변환
* 해당 물체가 없을 시 ROS가 종료되는 문제 발생
  : 예외처리를 통
```
