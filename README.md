# Capstone2021

## Project : Trailer Reversing Assistance System Considering Driver Input (feat.Capstone)  
  
지도 교수 : 백주훈 교수님  
참여 인원 : 강동운, 백승혁, 김예린, 이혜진
참여 기간 : 2021.1 ~ 2021.6

## Development Background 
- COVID-19의 장기화로 비교적 사람들과의 접촉이 적은 캠핑 트레일러를 이용한 여행 수요가 증가하고 있음 
- 트레일러를 부착한 차량은 일반 차량의 후진 주행방식과 달라 트레일러 운전이 익숙하지 않은 사람들은 많은 불편을 느낌 
- 자율주행 기능이 없는 차량에도 적용할 수 있는 트레일러 후진 보조 시스템을 제작
  
## Main Feature  
1. Gazebo Simulation 에서 후진 보조 시스템 성능 검증
2. 운전자 모델(1차 지연 입력필터)을 사용하여 입력 지연에 대한 시스템 안정도 평가
3. 지정한 경로를 추종하는 Algorithm 구현
4. 차선 인식을 통한 경로 추종 Algorithm 구현
5. 차량 - 트레일러 각도 측정모듈 제작

## Hardware
1. Wego-HT  
<img src="https://user-images.githubusercontent.com/52377778/119153611-5ccc1b00-ba8c-11eb-876d-c597b88c0a27.jpg" width="500" height="300" />   

2. Trailer
<img src="https://user-images.githubusercontent.com/52377778/119154036-c0564880-ba8c-11eb-8ff5-8c72b351eaad.jpg" width="200" height="250" />
<img src="https://user-images.githubusercontent.com/52377778/119154064-c77d5680-ba8c-11eb-8286-b965e17b3e2e.jpg" width="350" height="250" />  

3. 각도 측정 모듈
  
## Software
1. Path Planning (Pure Pursuit)

2. Vision Lookahead Point Estimation

3. Trailer Pose Estimation (feat.HT Model Odom data)

4. GUI Integration (Rear Camera + User Steering)
    
## System Architecture
<img src="https://user-images.githubusercontent.com/52377778/119155218-e6301d00-ba8d-11eb-9af1-eb3f155e81ed.PNG" width="800" height="500" />  
    
## Project GIF
Gazebo Simulation  
![UserSteeringFollowing_GazeboSimulation_gif](https://user-images.githubusercontent.com/52377778/119159621-635d9100-ba92-11eb-9732-b22b2bbad0a3.gif)

Real World Implementation  
![UserSteeringFollowing_Path2_gif](https://user-images.githubusercontent.com/52377778/119159617-6062a080-ba92-11eb-881e-7c8f3d8d3cdc.gif)







