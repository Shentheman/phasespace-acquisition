# PhaseSpace

## Launching robot with fake localization
- `roslaunch yuri_launch despot.launch`

## Launching phasespace
* `roslaunch phasespace_acquisition phasespace_publisher.launch`
  - Note that if you are running `despot.launch`, `phasespace_publisher.launch` might fail due to communication problems
  - After you physically connect the robot with phasespace, SSH into the robot router becomes very slow. So you should open all the necessary terminals with SSH **before** connecting to phasespace.
  - If connection keeps failing, then you can try reconnect the ethernet cable physically or re-Enable networking in Ubuntu.
    + Sometimes when you switch the connection of the robot network from the Internet to PhaseSpace system, `phasespace_publisher.launch` might fail and you might have to try physical Ethernet re-connections and reboot the Ubuntu network manager.
      + In order to avoid this trouble, please first disconnect physically the Ethernet between the robot system and the Internet, shut down Ubuntu network, wait 10 seconds, reconnect the Ethernet physically between the robot system and the PhaseSpace system. Usually in this sequence, `phasespace_publisher.launch` will work in the first try.
* `rosrun phasespace_acquisition msg_2_tf.py`
  - This will output the left LED on the palm as `phasespace/palm/left`
  * `rosrun phasespace_acquisition msg_2_tf_avg_palm.py`
    - This will output the center of the 2 LEDs on the palm as `phasespace/palm`

## Robot base localization
* In the phasespace environment, we don't have the markers to track the real-time pose of the robot
* Therefore, we will hard code the pose of the robot base and **make sure that the robot never moves during the experiments**
* Technically speaking, we will keep publishing the pose of the robot base `/odom_combined` in the frame of `map`.
  - `rosrun phasespace_acquisition fake_localization.py`

### Calibration pre-defined robot localization
* The calibration information is in `./config/fake_localization.yaml`
  - `map_2_odomCombined_trans` is the robot pose (usually `/odom_combined`) in the frame of `map`
* `roslaunch yuri_launch despot.launch`
* `roslaunch phasespace_acquisition phasespace_publisher.launch`
* `rosrun phasespace_acquisition msg_2_tf.py`
* `rosrun phasespace_acquisition localization_calibration.py`
* This will print out the translation from `map` to the phasespace marker named as `PHASESPACE_ID_FOR_CALIBRATION` in `localization_calibration.py`.
* You can place that LED onto the small red taped dot on the robot base, which is `odom_combined`
* Then you can read the translation printed by `localization_calibration.py` and put the translation in `./config/fake_localization.yaml`
  - Note that since usually we cannot place the LED glove under the robot base, we have to place it on the top of the robot base during calibration. So the translation of the LED represents the center of the top of the robot base. However, the `odom_combined` is the center of the bottom of the robot base. Therefore, here you have to manually make the last element of the translation (z) to be `0`.
* For orientation, you have to try the euler angles by yourself. Usually you can align the robot so that you only need to rotate `90` or `180`


# Finding out the translation of an object
* `roslaunch yuri_launch despot.launch`
* `roslaunch phasespace_acquisition phasespace_publisher.launch`
* `rosrun phasespace_acquisition fake_localization.py`
* `rosrun phasespace_acquisition msg_2_tf.py`
---
* `rosrun phasespace_acquisition object_translation_calibration.py`
  * This will print the TF from the `BASE_FRAME` (usually `/odom_combined`) to `PHASESPACE_ID_FOR_CALIBRATION` which are both defined in `object_translation_calibration.py`