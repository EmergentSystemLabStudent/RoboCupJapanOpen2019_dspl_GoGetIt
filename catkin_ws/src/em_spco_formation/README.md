# 'em_spco_formation' Package

The `em_spco_formation` enables the formation of Spatial Concepts.

*   Maintainer: Yuki Katsumata ([yuki.katsumata@em.ci.ritsumei.ac.jp](mailto:yuki.katsumata@em.ci.ritsumei.ac.jp)).
*   Author: Yuki Katsumata ([yuki.katsumata@em.ci.ritsumei.ac.jp](mailto:yuki.katsumata@em.ci.ritsumei.ac.jp)).

**Content:**

*   [Launch](#launch)
*   [Files](#files)

## Launch

*   `em_spcof_data_srv.launch`: Generate the data for Spatial Concept Formation.
*   `em_spcof_learn_srv.launch`: Learn Spatial Concept Formation.
*   `em_spcof_name2place.launch`: Estimate place by name using Learned Spatial Concept.
*   `em_spcof_place2name.launch`: Estimate name by place using Learned Spatial Concept.

## Files

*   `__init__.py`: Initialization of topics and parameters.
*   `em_spcof_data.py`: Service for data generation.
*   `em_spcof_data_image.py`: Generate the image dataset (feature by CNN) for Spatial Concept Formation.
*   `em_spcof_data_yolo.py`: Generate the image dataset (feature by BoO of YOLO) for Spatial Concept Formation.
*   `em_spcof_data_pose.py`: Generate the pose dataset for Spatial Concept Formation.
*   `em_spcof_data_word.py`: Generate the word dataset for Spatial Concept Formation.
*   `em_spcof_learn.py`: Learn Spatial Concept Formation.
*   `em_spcof_name2place.py`: Estimate place by name using Learned Spatial Concept.
*   `em_spcof_place2name.py`: Estimate name by place using Learned Spatial Concept.
*   `em_spcof_rviz.py`: View in Rviz.
