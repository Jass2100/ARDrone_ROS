# ARDrone_ROS
## 1. Обзорная часть
В данной части описан порядок выполнения проекта по предмету: Программное обеспечение мехатронных и робототехнических систем. Проект носит название: Управление квадрокоптером Parrot ARDrone 2.0, используя framework ROS. 

Проект был разбит на три основные части: 
1)	Написание класса для управления и получения одометрии квадрокоптера ARDrone 2.0 компании Parrot, используя фреймворк - ROS.
2)	Написание программы для управления полета квадрокоптера по заранее заданным координатам. Тестирование в симуляторе Gazebo.
3)	Переход от математической модели в Gazebo к реальной модели квадрокоптера Parrot ARDrone 2.0. Полёт квадрокоптера по квадрату.

Далее каждый из приведённых пунктов будут описываться более подробно.
## Написание класса для управления и получения одометрии квадрокоптера ARDrone 2.0 компании Parrot, используя фреймворк - ROS
На данном этапе необходимо:
1) провести работу, которая связана с подключением квадрокоптера к компьютеру по локальной сети Wi-Fi. Замерить и учесть в дальнейшем задержки, которые будут возникать по причине такой связи. 
2) Вторым пунктом является поиск пакета, который будет связывать три звена: ROS, компьютор и наш квадрокоптер воедино. Будет рассматриваться пакет ardrone_autonomy. Ссылка на wiki: http://ardrone-autonomy.readthedocs.io/en/latest/
3) Третий пункт - это непосредственно само программирование, которое будет осуществляться на языке C++, где необходимо будет реализовать простые поля класса Drone:

   * void Drone::Drone()
  
   * void Drone:: Drone()
  
   * void Drone::yaw_rotate(double persentage_of_cmd_vel)
  
   * void Drone::linear_x(double persentage_of_cmd_vel)
  
   * void Drone::linear_y(double persentage_of_cmd_vel)
  
   * void Drone::reset()
  
## Написание программы для управления полета квадрокоптера по заранее заданным координатам. Тестирование в симуляторе Gazebo
В данной части необходимо будет изучить сумулятор gazebo, в котором уже находится математическая модель квадрокоптера Parrot AARDrone 2.0, а также написать программу, которая будет управлять полетом квадрокоптера по точкам, опираясь на инерционную систему. Также использование в своей работе пакет tum_ardrone, который был предложен ребятами из Технического Университета Мюнхена для сверки по визуальной навигации, которая основана на алгоритме PTAM, который в свою очередь опирается на монокулярный SLAM. Написание функции для калибровки гироскопа.
## Переход от математической модели в Gazebo к реальной модели квадрокоптера Parrot ARDrone 2.0. Полёт квадрокоптера по квадрату
На данном этапе необходимо будет перенести все свои знания, которые были получены в симуляторе, на реальную модель квадрокоптера. В данной части буду в большинстве опыты в "полевых условиях"
## 2. CMakeFiles

In this part we connect ROS and Qtcreator( QtGui, QtCore, Qt Designer).

   2.1. Check cmake version:

    cmake_minimum_required(VERSION 2.8.3)
   2.2. Set project name:    

       set(PROJECT YOUR_PROJECT_NAME)
       project(${PROJECT})
   2.3. Set sources files:    

       set(SOURCES
             main.cpp
             mainwindow.cpp
             ControllerNode.cpp
          )
          
   2.4. Set moc headers files:    

       set(MOC_HEADERS
             mainwindow.cpp
             ControllerNode.hpp
          )
          
   2.5. Set UI's files( if you need it):    

       set(UIS
             mainwindow.ui
          )
   2.6. Next lines needed for building all Qt projects:

       find_package(Qt4 REQUIRED)
       include(${QT_USE_FILE})
       add_definitions(${QT_DEFINITIONS})
       include_directories(${CMAKE_BINARY_DIR})
       
   2.7. Find ROS packages. Dependencies on other catkin packages can be added in a COMPONENTS section on
this line( we added roscpp, for example):

       find_package(catkin REQUIRED COMPONENTS roscpp)
   2.8. Using Qt meta-system (precompiler):

       QT4_WRAP_UI(UI_HEADERS ${UIS})
       QT4_WRAP_CPP(MOC_SRCS ${MOC_HEADERS})
   2.9. Compile:

       add_executable(${PROJECT} ${SOURCES} ${MOC_SRCS} ${UI_HEADERS})
   2.10. Build it (link libraries):

       target_link_libraries(${PROJECT} ${QT_LIBRARIES} ${catkin_LIBRARIES})

