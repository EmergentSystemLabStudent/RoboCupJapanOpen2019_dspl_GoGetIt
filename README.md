# Go Get It

### Overview  

Go get it task for OIT and RITS2019 in RoboCup Japan Open

URL : https://emlab.jimdo.com/multimedia/

###### Training Phase

DoorOpenによって競技がスタートすると，ロボットは環境の地図を作成する． ロボットはPerson Tracking（Follow me）を行い，オペレータの後をついて移動し，場所の名前情報を教示してもらう．ロボットは環境内で得たマルチモーダル情報を用い，石伏らの手法[1][2]に谷口らの手法[3]を組み合わせ，場所の名前情報・位置情報・画像情報を用いて場所のカテゴリゼーションを行う．石伏らの手法は画像情報と位置情報を用いて場所のカテゴリゼーションを行った手法であり，谷口らの手法は言語情報と位置情報から場所を表す名前と場所のカテゴリを同時に獲得する手法である．  
また，bring meのために画像認識モジュールを用いた場所と物体の関連付けを行う．ロボットに場所概念を教示すると同時にロボット頭部のセンサから画像を取得し，取得した画像から画像認識モジュールを用いて認識できる物体ラベルをその場所に存在する物体の名前として記録する．画像認識モジュールには深層学習モデル「YOLO v3」[4]をcoco dataset[5]で訓練したモデルを用いた。
###### Test Phase

命令文から場所の名前情報を取り出し，その場所の名前が表す場所のカテゴリの尤度に基づいて場所のカテゴリを特定し，その場所のカテゴリが持つガウス分布に基づいて移動先を決定する．  
移動した後，画像認識モジュールを用いてロボット頭部のセンサから取得した画像から検出した物体とTraining Phaseで検出した物体の双方に含まれているものを把持対象とする．物体はcoco datasetでのラベルしか持っていないため，人間の認識と合致させるためにロボットはオペレータに対して「May I take the object?」のような問いかけるインタラクションを行う．応答の結果によってロボットは取りに行く物体を決定することができる．  
マニピュレーションはMoveIt![6]を用いており，物体の世界座標を目標にロボットの手先座標を移動させる制御を行なっている．物体の座標へ一度のプランニングで移動すると軌道の自由度が高く失敗する確率が高いため，ロボットと物体の座標の直線上の物体から30cmの地点をwaypointとし，2回軌道生成を行うことで成功率を向上させている．

### Description

###### Start

1. `roslaunch hsr_launch em_follow_me_default.launch`

2. hsr_launchの以下のlaunchファイルを全て起動する
`as_julius_default.launch`
`tmc_moveit_default.launch`
`hsr_common_cv_detect_object.launch`
`em_speech_default.launch`
`em_map_record_default.launch`
`em_spco_formation_default.launch`

3. `roslaunch hsr_launch flexbe_app_default.launch`

4. flexbe[7]のGUIが展開されるので`Load Behavior`から`Go_get_it_unknown_environment2019`を選択する．

5. `Runtime Control`の画面に移り，`Start Execution`のボタンを押すとロボットの動作を開始することができる

### Paper

1. 石伏智，谷口彰，萩原良信，高野敏明，谷口忠大「自己位置と高次特徴量を用いた教師なし場所領域学習」第30回人工知能学会全国大会(JSAI2016)，2016年5月，福岡，同上論文集，2I3-4.
2. Satoshi Ishibushi, Akira Taniguchi, Toshiaki Takano, Yoshinobu Hagiwara and Tadahiro Taniguchi: “Statistical Localization Exploiting Convolutional Neural Network for an Autonomous Vehicle”, 41st Annual Conference of the IEEE Industrial Electronics Society (IECON’15), Nov. 9-12, 2015 in Yokohama (Japan). The proceedings of IECON’15, pp. 1369-1375.
3. Akira Taniguchi, Tadahiro Taniguchi, and Tetsunari Inamura: “Spatial concept acquisition for a mobile robot that integrates self-localization and unsupervised word discovery from spoken sentences,” IEEE Transactions on Cognitive and Developmental Systems, vol. 8, no. 4, pp. 285–297, 2016.
4. Joseph Redmon, Ali Farhadi. YOLOv3: An Incremental Improvement. arXiv 2018.
5. Lin, Tsung-Yi, et al. "Microsoft coco: Common objects in context." European conference on computer vision. Springer, Cham, 2014.
6. Chitta, Sachin, Ioan Sucan, and Steve Cousins. "Moveit![ros topics]." IEEE Robotics & Automation Magazine 19.1 (2012): 18-19.
7. ROS.org "FlexBE" http://wiki.ros.org/flexbe
