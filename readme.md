## 1. webots install & MATLAB Installation
webots 2025a 버전 다운  
<img src="./resource/readme/image2.png" alt="alt text" width="300"/>

https://cyberbotics.com/

## 2. webots & matlab integration
### 프로젝트 폴더 생성
webots 설치한 이후
>C:\Webots\projects\robots\softbank\  

이 경로의 nao 폴더 전체를 본인의 프로젝트 폴더로 이동시킨다.
nao\worlds\ 의 nao_matlab.wbt 파일 실행

### matlab cpp compiler 설치
webots 시뮬레이터는 cpp 기반이기 때문에 matlab cpp compiler 필요  
"On Windows the MATLAB MinGW-w64 C/C++ Compiler needs to be installed in addition to MATLAB."
![alt text](./resource/readme/image.png)
https://kr.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler

## 3. webots 환경(robot, obstacle) 세팅
2번까지 되었을 때 파일중에 nao_matlab_v2.wbt 와 같이 world 파일을 nao\worlds 안에 복사하여 실행하거나 본인의 새로운 로봇, 장애물을 설치하고자 할 때에는 아래 과정 참고

webots 의 경우 화면 왼쪽에서 트리 구조를 통해 로봇의 센서, 센서 위치 등의 환경을 수정할 수 있다.
트리에서 Nao "NAO" 를 오른쪽 마우스 클릭 하여 "convert root to base nodes" 를 하여 로봇의 센서 정보를 수정할 수 있도록 잠금 해제  

1. Lidar 센서 추가

![alt text](./resource/readme/image3.png)![alt text](./resource/readme/image4.png)

Robot "NAO" -> children 에 Lidar 센서 추가
Lidar 설정 -> tiltAngle -0.1, horizontalResolution 128, numberOfLayers 2, near 0.05, minRange 0.05, maxRange 8, defaultFrequency2


2. Camera resolution 변경 (하지 않아도 됨, default 는 저화질)
webots 로봇의 트리에서
Robot "NAO" -> children -> DEF HeardYaw Hinge2Joint -> endPoint Solid -> Camera "CameraTop" 
width 640, height 480

![alt text](./resource/readme/image7.png)![alt text](./resource/readme/image4.png)

3. 장애물 추가 방법
트리에서 Floor "floor" 를 클릭하여 포커스가 변경된 상태에서 상단바의 + 버튼 -> PROTO nodes (Webots Projects) ->  objects 에서 원하는 장애물 설치

## 4. offline 작업
**``우선 webots 의 로봇은 기본적으로 nao\controllers\nao_matlab\nao_matlab.m 파일로 실행된다. (트리에서 변경 가능)
task 에 따라 사용되는 nao_matlab_~.m 코드를 nao_matlab.m 파일에 복사 붙여넣기 한 뒤 webots 에서 reload 한뒤 실행하면 변경된 코드로 실행된다.``**

![alt text](./resource/readme/image5.png)

### 2.1. Object Detection (YOLO v4)(Deep Learning)
#### 2.1.1. dataset 생성
1. nao_matlab_mk_img_dataset.m -> nao_matlab.m  
image 저장 -> frame_*.png 형태로 저장  
이미지중 기울어짐, 로봇의 이상 위치와 같은 잘못된 이미지 삭제  
frame_namer.mlx 파일 활용하여 모든 이미지의 frame 이 연속되도록 변경  
```
Renamed: frame_1.png -> frame_0001.png
Renamed: frame_2.png -> frame_0002.png
Renamed: frame_3.png -> frame_0003.png
Renamed: frame_4.png -> frame_0004.png
Renamed: frame_8.png -> frame_0005.png
Renamed: frame_9.png -> frame_0006.png
...
Renamed: frame_292.png -> frame_0220.png
Renamed: frame_293.png -> frame_0221.png
```

2. 비디오 레이블 지정기
비디오 레이블 지정기 앱열어서 비디오 시퀀스 불러오기  
![alt text](./resource/readme/image6.png)  

장애물 종류에 따라 라벨 생성후 라벨링  

#### 2.1.2. object detection model 생성 및 학습
1. yolo_model_make.mlx  
첫번째 코드 matFile = "./~"; 에서 경로 설정을 하고 차례대로 실행  
포함된 코드  
- 모델 학습을 위한 data 구조 생성 및 모델 학습.
- 학습된 모델의 정확도 확인
- 임의의 사진에 적용하여 object detection 결과 확인 가능
- 신경망 구조 확인
- 최종 모델 파일로 저장 -> "detector_nao_0630.m"

2. object detetection model 경량화
future works

### 2.2. Path Planning (DDPG)(Reinforcement Learning)

#### 2.2.1. lidar 데이터 기반 장애물 맵 생성 및 강화학습 경로 생성 모델 학습

1. nao_matlab_mk_lidar_dataset.m -> nao_matlab.m
코드를 실행하면 레이더 데이터가 시각적으로 표시되고 레이더 데이터가 csv 파일로 저장된다.  
![alt text](./resource/readme/image8.png)  

2. avoid_obstacle_lidar.mlx


3. avoid_obstacle_robot.mlx

## 5. online 작업
