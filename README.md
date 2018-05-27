# ARDrone_ROS

## Видео отчёт проекта

[Работа первого алгоритма](https://drive.google.com/file/d/1m6HPL7GIhevR-G7bHznxpd58TfcCb0_k/view?usp=sharing "Видео")

[Работа второго алгоритма](https://drive.google.com/file/d/1QUrBSpx2QuXzDl3aAOyr9Bl6DExjVX6l/view?usp=sharing "Видео")

## Введение 
В наши дни все большую популярность набирают беспилотные летательные аппараты, а именно мультикоптеры или, как их чаще всего называют, квадрокоптеры - это летательный аппарат построенный по вертолётной схеме с тремя и более несущими винтами. Обычно данные устройства имеют грузоподъемность от 500г до 2-3 кг, что позволяет поднять в воздух небольшую фото или видеокамеру (обычно экшн-камера в более дешёвых моделях, либо зеркальные камеры в профессиональных). Но сейчас всё больше и больше начинают использовать квадрокоптеры в технических сферах, например, для построения карты местности, используя монокулярную камеру, или для слежения за выбранным объектом. В связи с этим в интернете появляется довольно много материала по данной тематике, так как это с одной стороны занимательная, а с другой достаточно перспективная отрасль. Как следствие, данные ЛА стали все чаще использоваться не только для съемок фото- и видеоматериалов, но и для транспортировки и доставки легких грузов. Особую сложность составляет полет по траектории в закрытых помещениях или места, в которых GPS по какой-либо причине не доступна. С развитием технического зрения все большую популярность для определения положения объекта в неизвестном пространстве набирает визуальная навигация, основанная на монокулярной камере в качестве основного датчика и, следовательно, не нуждающаяся во внешних датчиках, таких как: GPS или визуальные маркеры. Это позволяет достаточно точно позиционировать недорогой квадрокоптер, на котором отсутствует или не доступна система GPS. В то же время у данного метода есть некоторые ограничения. Одним из таких является необходимость осуществлять вычисления, связанные с обработкой данных с камеры, на внешнем устройстве, например, ноутбуке, так как они требуют затраты достаточно больших ресурсов. Как следствие, необходимо наличие беспроводной локальной связи. В данной работе предлагается рассмотреть алгоритм позиционирования квадрокоптера, основанный на PTAM (Parallel Tracking and Mapping), который был предложен Якобом Энгелем, Юргеном Штурмом и Даниэлем Кремерсом Технического университета Мюнхена, Германия. Также два алгоритма полета по траектории.

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

## Описание класса Drone

Для управления квадрокоптером под ROS'ом был написан простой класс управления - Drone. В данном разделе будет его краткое описание.
+ **void Drone::Drone()**  
Конструктор инициализирует ROS, назначает топики, с которых будет брать данный и в которые будет посылать команды


+ **void Drone::~Drone()**   
Деструктор, который запускает закрытие процесса ROS и ждёт его завершения

+ **void Drone::yaw_rotate(double persentage_of_cmd_vel)**   
Поворачивает квадрокоптер относительно оси z (рысканья), на вход функции подается процент от максимальной скорости вращения

+ **void Drone::linear_x(double persentage_of_cmd_vel)**     
Осуществляет движение квадрокоптера по оси x. На вход функции подается процент от максимальной скорости движения

+ **void Drone::linear_y(double persentage_of_cmd_vel)**   
Осуществляет движение квадрокоптера по оси x. На вход функции подается процент от максимальной скорости движения

+ **void Drone::reset()**     
Команда осуществляет рестарт квадрокоптера

+ **void Drone::take_off()**     
Команда осуществляет взлёт квадрокоптера

+ **void Drone::land()**     
Команда осуществляет посадку квадрокоптера

+ **void Drone::hover()**     
Заставляет квадрокоптер зависнуть в заданной точке

+ **Drone::Euler_angle Drone::to_euler_angle()**     
Переводит кватернион в углы Эйлера и возвращает в виде структуры Drone::Euler_angle

+ **Drone::Position Drone::get_position()**     
Возвращает текущие координаты по осям XYZ в виде структуры Drone::Position

+ **void Drone::get_odometry(const nav_msgs::Odometry & msg)**       
Данная функция возвращает одометрию квадрокоптера

+ **double Drone::gyro_calibration()**     
Данная функция производит калибровку квадрокоптера

+ **Navdata Drone::get_navdata()**     
Данная функция возвращает данные Navdata в виде структуры Navdata

При полете в закрытых помещениях встает вопрос о навигации квадрокоптера, так как использование GPS является достаточно неточным способом позиционирования квадрокоптера в помещении из-за его погрешности, которая считается достаточно точной на открытых пространствах. В данной работе предлагается рассмотреть алгоритм позиционирования квадрокоптера, основанный на PTAM (Parallel Tracking and Mapping), который был предложен Якобом Энгелем, Юргеном Штурмом и Даниэлем Кремерсом Технического университета Мюнхена, Германия.

В данной работе мы будем рассматривать движение квадрокоптера в плоскости XOY, координата Z будет фиксирована и задана в самом начале программы.

## Алгоритмы полета квадрокоптера в точку   
Эскиз квадрокоптера, который движется в заданную точку в пространстве, представлен ниже     
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/cxem.jpg "")     
+ Использование скоросте по каждым из осям (омни привод)
+ Использование Функции Лапласса   
### Использование Функции Лапласса 
  Опишем математическую модель     
  ![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/model_1.jpg "")   
  Фактически, квадрокоптер может полностью управляться с помощью значений угловой и линейной скорости, поэтому нужно найти такие их значения, чтобы выполнялось поставленное условие задачи. Для этого в статье предлагается воспользоваться аппаратом функции Ляпунова. Это будет квадратичная функция, включающая в себя расстояние до цели и курсовой угол:      
  ![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/formula_1.jpg "") 
  
  производная по времени должна быть не положительна для того, чтобы расстояние до цели и курсовой угол не возрастали. Производная выглядит следующим образом:   
  ![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/formula_2.jpg "")    
  Выразив производную через математическую функцию, предложенную выше, получаем:     
  ![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/formula_3.jpg "")    
  Эта производная отрицательна определена, если в качестве управляющего воздействия взять следующие значения скоростей:     
  ![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/formula_4.jpg "")     
  Всё это позволяет квадрокоптеру достичь своей цели, но пока лишь в отсутствии препятствий.   
### Использование скоросте по каждым из осям (омни привод)   
Опишем математическую модель    
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/formula_5.jpg "")      
## Тестирование алгоритмов 
В данном разделе проводится проверка выше написанных алгоритмов в симуляторе Gazebo на модели квадрокоптера ARDrone 2.0 компании Parrot.    

Квадрокоптеру необходимо подняться в воздух в точке 0;0, после пролететь квадрат со стороной 1 метра, вернуться в исходное пложение и сесть.  

В данном алгоритме квадрокоптер достаточно точно достигал заданные точки, это можно проследить на рисунке. Практически все точки: (0;1), (1;1), (1;0), (0;0) были достигнуты, хотя в данном регуляторе используется только пропорциональная составляющая. Большим недостатком является большое время выполнения, которое составило 36.34 секунды, а также затрата места на разворот квадрокоптера.
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/x_coord_first.png  "")   
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/y_coord_first.png  "")  
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/xy_coord_first.png  "")    

В данном алгоритме были достаточно большие ошибки по достижению заданных точек, это можно проследить на рисунке \ref{pic:pic_16}. Тут также использовалась только пропорциональная составляющая регулятора. Плюсом данного метода является скорость выполнения программы, которая составила всего 13.2 секунды, что практически в три раза меньше, чем в первом методе.  
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/x_coord_second.png  "")   
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/y_coord_second.png  "")  
![](https://github.com/Jass2100/ARDrone_ROS/blob/master/pics/xy_coord_second.png  "")  

## Заключение

В данной работе было разобрано два алгоритма полета квадрокоптера по координатам и метод визуальной навигации, который был разработан сотрудниками Технического Университета Мюнхена, Германия. Также метод реализации на С++. По проделанным опытам можно сказать, что данный метод хорошо себя показал в определении положения в пространстве, если считать, что в регуляторах была всего одна составляющая - пропорциональная. В дальнейшем планируется доработать алгоритмы, добавив в них интегральную и дифференциальную составляющие, а после перенести все разработки на реальную модель квадрокоптера ARDrone 2.0, о котором можно сказать, что он является одним из самых доступных беспилотных летательных аппаратов на данный момент для осуществления S.L.A.M'a и изучения основ строения и управления квадрокоптерами. Также данный БЛА обладает открытой библиотекой - SDK 2.0, что позволяет производить все возможные настройки и улучшать точность стабилизации посредством усовершенствования ПИД регулятора и САУ в общем. Если брать во внимание растущую популярность квадракоптеров, то улучшение программной и механической составляющей является очень перспективным направлением. Не смотря на все свои плюсы, квадротор имеет также и ряд минусов. Во-первых, это слишком малая грузоподъемность, которая ограничивается тремястами граммами. Во-вторых, достаточно небольшое время в полете, примерно 12 минут\cite{Parrot}, в реальности при наличии множества внешних воздействий это значение падает до 7 минут. В-третьих, чтобы достичь стабилизации квадрокоптера в воздухе на протяжении всего полета необходимо приложить немало усилий и произвести дополнительные действия для более точной работы датчиков ARDrone 2.0. И это только одни из самых явных минусов. Но не смотря на это, данный квадрокоптер является одним из самых качественных продуктов в своей ценовой категории.
