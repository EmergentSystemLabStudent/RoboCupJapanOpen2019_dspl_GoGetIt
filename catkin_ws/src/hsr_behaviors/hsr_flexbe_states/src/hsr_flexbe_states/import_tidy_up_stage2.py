from geometry_msgs.msg import Pose, Point

#v3_COCO
unknown_list = []
known_dict = {'cup': 'coffee table', 'vase': 'wall shelf', 'pottedplant': 'wall shelf', 'bottle': 'trash'}

#Fast R-CNN
# unknown_list = []
# known_dict = {'blue_cup': 'kitchen unit', 'pink_cup': 'kitchen unit', 'mineral_water': 'trash', 'mixed_nuts': 'coffee table', 'yellow_biscuits': 'food cabinet', 'green_drink': 'coffee table', 'orange_drink': 'coffee table', 'orange_biscuits': 'food cabinet', 'ketchup': 'trash', 'oolong_tea': 'coffee table', 'tomato': 'trash', 'canned_mustard': 'trash', 'blueberry_drink': 'trash', 'pink_biscuits': 'food cabinet'}

table_move = Pose()
table_move.position.x = 2.8485027922
table_move.position.y = 0.251997413128
table_move.orientation.z = 0.712476857409
table_move.orientation.w = 0.701695608976

sofa_move = Pose()
sofa_move.position.x = 2.80525590518
sofa_move.position.y = -1.89861971785
sofa_move.orientation.z = 0.305767380657
sofa_move.orientation.w = 0.952106248759

kitchen_move = Pose()
kitchen_move.position.x = 2.34321954479
kitchen_move.position.y = 1.66423572861
kitchen_move.orientation.z = 0.999953752564
kitchen_move.orientation.w = 0.00961731426391

kitchen_unit_move = Pose()
kitchen_unit_move.position.x = 2.8573673313
kitchen_unit_move.position.y = -2.36804470873
kitchen_unit_move.orientation.z = -0.00576926111107
kitchen_unit_move.orientation.w = 0.999983357675

food_cabinet_move = Pose()
food_cabinet_move.position.x = 3.32159523213
food_cabinet_move.position.y = 2.62597206041
food_cabinet_move.orientation.z = 0.98249401263
food_cabinet_move.orientation.w = 0.186294162944

coffee_table_move = Pose()
coffee_table_move.position.x = 2.8573673313
coffee_table_move.position.y = -2.36804470873
coffee_table_move.orientation.z = -0.00576926111107
coffee_table_move.orientation.w = 0.999983357675

wall_shelf_move = Pose()
wall_shelf_move.position.x = 2.22536623967
wall_shelf_move.position.y = -1.13751487209
wall_shelf_move.orientation.z = -0.999901255876
wall_shelf_move.orientation.w = 0.0140527042482

trash_move = Pose()
trash_move.position.x = 2.22316673855
trash_move.position.y = -1.17237509623
trash_move.orientation.z = -0.960890041148
trash_move.orientation.w = 0.276930187634


kitchen_unit_arm_0 = Pose()
kitchen_unit_arm_0.position.x = 3.596096562
kitchen_unit_arm_0.position.y = -2.32463396171
kitchen_unit_arm_0.position.z = 0.492700318824
kitchen_unit_arm_0.orientation.x = -0.74722237492
kitchen_unit_arm_0.orientation.y = 0.0362248856962
kitchen_unit_arm_0.orientation.z = -0.663004283875
kitchen_unit_arm_0.orientation.w = -0.0277812821672

kitchen_unit_arm_1 = Pose()
kitchen_unit_arm_1.position.x = 3.596096562
kitchen_unit_arm_1.position.y = -2.32463396171
kitchen_unit_arm_1.position.z = 0.492700318824
kitchen_unit_arm_1.orientation.x = -0.74722237492
kitchen_unit_arm_1.orientation.y = 0.0362248856962
kitchen_unit_arm_1.orientation.z = -0.663004283875
kitchen_unit_arm_1.orientation.w = -0.0277812821672

kitchen_unit_arm = [kitchen_unit_arm_0, kitchen_unit_arm_1]

food_cabinet_arm_0 = Pose()
food_cabinet_arm_0.position.x = 3.65512815627
food_cabinet_arm_0.position.y = 3.56717779698
food_cabinet_arm_0.position.z = 0.817446458053
food_cabinet_arm_0.orientation.x = -0.494128871484
food_cabinet_arm_0.orientation.y = -0.399289576562
food_cabinet_arm_0.orientation.z = -0.609078049728
food_cabinet_arm_0.orientation.w = 0.474793030439

food_cabinet_arm_1 = Pose()
food_cabinet_arm_1.position.x = 3.33774477397
food_cabinet_arm_1.position.y = 3.30136128276
food_cabinet_arm_1.position.z = 0.817228354594
food_cabinet_arm_1.orientation.x = -0.489066569875
food_cabinet_arm_1.orientation.y = -0.405888056743
food_cabinet_arm_1.orientation.z = -0.601937284493
food_cabinet_arm_1.orientation.w = 0.483466939056

food_cabinet_arm = [food_cabinet_arm_0, food_cabinet_arm_1]

coffee_table_arm_0 = Pose()
coffee_table_arm_0.position.x = 3.596096562
coffee_table_arm_0.position.y = -2.32463396171
coffee_table_arm_0.position.z = 0.492700318824
coffee_table_arm_0.orientation.x = -0.74722237492
coffee_table_arm_0.orientation.y = 0.0362248856962
coffee_table_arm_0.orientation.z = -0.663004283875
coffee_table_arm_0.orientation.w = -0.0277812821672

coffee_table_arm_1 = Pose()
coffee_table_arm_1.position.x = 3.596096562
coffee_table_arm_1.position.y = -2.32463396171
coffee_table_arm_1.position.z = 0.492700318824
coffee_table_arm_1.orientation.x = -0.74722237492
coffee_table_arm_1.orientation.y = 0.0362248856962
coffee_table_arm_1.orientation.z = -0.663004283875
coffee_table_arm_1.orientation.w = -0.0277812821672

coffee_table_arm_2 = Pose()
coffee_table_arm_2.position.x = 2.38403152471
coffee_table_arm_2.position.y = -2.25470141293
coffee_table_arm_2.position.z = 0.499277609158
coffee_table_arm_2.orientation.x = -0.710131426215
coffee_table_arm_2.orientation.y = 0.0211617923664
coffee_table_arm_2.orientation.z = -0.703394527896
coffee_table_arm_2.orientation.w = -0.0223980841013

coffee_table_arm_3 = Pose()
coffee_table_arm_3.position.x = 4.76159472328
coffee_table_arm_3.position.y = 3.67724383653
coffee_table_arm_3.position.z = 0.557124848554
coffee_table_arm_3.orientation.x = -0.256117962742
coffee_table_arm_3.orientation.y = -0.578406589313
coffee_table_arm_3.orientation.z = -0.231688751168
coffee_table_arm_3.orientation.w = 0.739032968942

coffee_table_arm = [coffee_table_arm_0, coffee_table_arm_1, coffee_table_arm_2, coffee_table_arm_3]


wall_shelf_arm_0 = Pose()
wall_shelf_arm_0.position.x = 1.315451581
wall_shelf_arm_0.position.y = -1.18978154981
wall_shelf_arm_0.position.z = 0.595082135769
wall_shelf_arm_0.orientation.x = -0.0319092622219
wall_shelf_arm_0.orientation.y = -0.74910147739
wall_shelf_arm_0.orientation.z = -0.0239397549558
wall_shelf_arm_0.orientation.w = 0.661253101081

wall_shelf_arm_1 = Pose()
wall_shelf_arm_1.position.x = 1.315451581
wall_shelf_arm_1.position.y = -1.18978154981
wall_shelf_arm_1.position.z = 0.595082135769
wall_shelf_arm_1.orientation.x = -0.0319092622219
wall_shelf_arm_1.orientation.y = -0.74910147739
wall_shelf_arm_1.orientation.z = -0.0239397549558
wall_shelf_arm_1.orientation.w = 0.661253101081

wall_shelf_arm_2 = Pose()
wall_shelf_arm_2.position.x = 9.28815307279
wall_shelf_arm_2.position.y = 3.67063876836
wall_shelf_arm_2.position.z = 0.880612643699
wall_shelf_arm_2.orientation.x = -0.436171398523
wall_shelf_arm_2.orientation.y = -0.480103303396
wall_shelf_arm_2.orientation.z = -0.520035880246
wall_shelf_arm_2.orientation.w = 0.555713966386

wall_shelf_arm_3 = Pose()
wall_shelf_arm_3.position.x = 9.37004957325
wall_shelf_arm_3.position.y = 3.74935443811
wall_shelf_arm_3.position.z = 0.894470365342
wall_shelf_arm_3.orientation.x = -0.484163615699
wall_shelf_arm_3.orientation.y = -0.431710788786
wall_shelf_arm_3.orientation.z = -0.575241010238
wall_shelf_arm_3.orientation.w = 0.498306299598

wall_shelf_arm_4 = Pose()
wall_shelf_arm_4.position.x = 9.04092497575
wall_shelf_arm_4.position.y = 3.46156903817
wall_shelf_arm_4.position.z = 1.17199589111
wall_shelf_arm_4.orientation.x = -0.516044233684
wall_shelf_arm_4.orientation.y = -0.393850408872
wall_shelf_arm_4.orientation.z = -0.611386463422
wall_shelf_arm_4.orientation.w = 0.452533751953

wall_shelf_arm_5 = Pose()
wall_shelf_arm_5.position.x = 8.93421918166
wall_shelf_arm_5.position.y = 3.7325566013
wall_shelf_arm_5.position.z = 1.21225233981
wall_shelf_arm_5.orientation.x = -0.446940460288
wall_shelf_arm_5.orientation.y = -0.471227893248
wall_shelf_arm_5.orientation.z = -0.531377826578
wall_shelf_arm_5.orientation.w = 0.543898982352

wall_shelf_arm = [wall_shelf_arm_0, wall_shelf_arm_1, wall_shelf_arm_2, wall_shelf_arm_3, wall_shelf_arm_4, wall_shelf_arm_5]

trash_arm = Pose()
trash_arm.position.x = 1.62000735289
trash_arm.position.y = -1.64580661954
trash_arm.position.z = 0.492701192927
trash_arm.orientation.x = 0.194537815047
trash_arm.orientation.y = -0.722363176277
trash_arm.orientation.z = 0.177000363176
trash_arm.orientation.w = 0.639544643876


table_xtion = Point()
table_xtion.x = 2.73103836996
table_xtion.y = 1.17606345
table_xtion.z = 0.734381768242

sofa_xtion = Point()
sofa_xtion.x = 3.53273123566
sofa_xtion.y = -1.33890494227
sofa_xtion.z = 0.433859452444

kitchen_unit_xtion = kitchen_unit_arm_0.position

food_cabinet_xtion = food_cabinet_arm_0.position

coffee_table_xtion = coffee_table_arm_0.position

wall_shelf_xtion = wall_shelf_arm_0.position

trash_xtion = trash_arm.position
